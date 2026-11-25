/* flipper_trail module: position time gps. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---- Last known position: lets the map work before (or without) a GPS fix ---- */
#define LASTPOS_PATH "/ext/GPS/lastpos.bin"
typedef struct {
    char magic[3];
    uint8_t version;
    int32_t lat_e7, lon_e7; /* version 2: 1e-7 degree */
} LastPos;
typedef struct {
    char magic[3];
    uint8_t version;
    float lat, lon; /* version 1 */
} LastPosV1;

static bool lastpos_load(double* lat, double* lon) {
    bool ok=false;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,LASTPOS_PATH,FSAM_READ,FSOM_OPEN_EXISTING)) {
        LastPos lp;
        if(storage_file_read(f,&lp,sizeof(lp))==sizeof(lp) && memcmp(lp.magic,"FLP",3)==0) {
            double a=0.0, b=0.0;
            bool valid=false;
            if(lp.version==2) {
                a=e7_to_deg(lp.lat_e7);
                b=e7_to_deg(lp.lon_e7);
                valid=true;
            } else if(lp.version==1) {
                LastPosV1 old;
                memcpy(&old,&lp,sizeof(old));
                a=(double)old.lat;
                b=(double)old.lon;
                valid=true;
            }
            if(valid && a>=(double)-90.0 && a<=(double)90.0 && b>=(double)-180.0 && b<=(double)180.0) {
                *lat=a;
                *lon=b;
                ok=true;
            }
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

static void lastpos_save(double lat, double lon) {
    LastPos lp={.magic={'F','L','P'},.version=2,.lat_e7=deg_to_e7(lat),.lon_e7=deg_to_e7(lon)};
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,LASTPOS_PATH,FSAM_WRITE,FSOM_CREATE_ALWAYS)) {
        storage_file_write(f,&lp,sizeof(lp));
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

/* Remember where we are: at most every 30 s and only after moving 10 m (force = on stop/exit). */
static void lastpos_tick(Trail* t, bool force) {
    if(!t->fix) return;
    uint32_t now=furi_get_tick();
    if(!force && t->saved_pos_valid && (uint32_t)(now-t->lastpos_tick)<30000) return;
    t->lastpos_tick=now;
    if(t->saved_pos_valid && dist_m(t->saved_lat,t->saved_lon,t->lat,t->lon)<10.0f) return;
    lastpos_save(t->lat,t->lon);
    t->saved_pos_valid=true;
    t->saved_lat=t->lat;
    t->saved_lon=t->lon;
}

static int tz_zone_offset(const TzZone* z, uint32_t utc) {
    int off=z->std_min;
    for(uint8_t i=0;i<z->n && i<TZ_MAX_TRANS;i++) {
        if(utc>=z->t[i]) off=z->off[i];
        else break;
    }
    return off;
}

static void tz_cell(const TzHeader* h, float lat, float lon, int* col, int* row) {
    int cx=(int)floorf((lon+180.0f)*(float)h->cols/360.0f);
    int cy=(int)floorf((lat+90.0f)*(float)h->rows/180.0f);
    if(cx<0) cx=0;
    if(cx>=(int)h->cols) cx=(int)h->cols-1;
    if(cy<0) cy=0;
    if(cy>=(int)h->rows) cy=(int)h->rows-1;
    *col=cx;
    *row=cy;
}

/* UTC offset (minutes) at this place and instant from tz.bin. Only touches the SD
   card when the grid cell changes. False if tz.bin is missing or unreadable. */
static bool tz_lookup(TzCache* c, float lat, float lon, uint32_t utc, int* off_min) {
    if(c->state==TzStateMissing) {
        if((int32_t)(furi_get_tick()-c->retry_tick)<0) return false;
        c->state=TzStateUnknown;
    }
    if(c->state==TzStateOk) {
        int col,row;
        tz_cell(&c->h,lat,lon,&col,&row);
        if(c->zone_ok && col==c->col && row==c->row) {
            *off_min=tz_zone_offset(&c->zone,utc);
            return true;
        }
    }
    bool ok=false;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(!storage_file_open(f,TZ_PATH,FSAM_READ,FSOM_OPEN_EXISTING)) {
        c->state=TzStateMissing;
        c->retry_tick=furi_get_tick()+10000;
    } else {
        if(c->state==TzStateUnknown) {
            if(storage_file_read(f,&c->h,sizeof(c->h))==sizeof(c->h) &&
               memcmp(c->h.magic,TZ_MAGIC,3)==0 && c->h.version==TZ_VERSION &&
               c->h.cols>0 && c->h.rows>0 && c->h.zone_count>0) {
                c->state=TzStateOk;
                c->zone_ok=false;
            } else {
                c->state=TzStateMissing;
                c->retry_tick=furi_get_tick()+60000;
            }
        }
        if(c->state==TzStateOk) {
            int col,row;
            tz_cell(&c->h,lat,lon,&col,&row);
            uint32_t grid=(uint32_t)sizeof(TzHeader)+(uint32_t)c->h.zone_count*(uint32_t)sizeof(TzZone);
            uint8_t idx=0;
            if(storage_file_seek(f,grid+(uint32_t)row*c->h.cols+(uint32_t)col,true) &&
               storage_file_read(f,&idx,1)==1 && idx<c->h.zone_count &&
               storage_file_seek(f,(uint32_t)sizeof(TzHeader)+(uint32_t)idx*(uint32_t)sizeof(TzZone),true) &&
               storage_file_read(f,&c->zone,sizeof(TzZone))==sizeof(TzZone)) {
                if(c->zone.n>TZ_MAX_TRANS) c->zone.n=TZ_MAX_TRANS;
                c->col=col;
                c->row=row;
                c->zone_ok=true;
                *off_min=tz_zone_offset(&c->zone,utc);
                ok=true;
            } else {
                c->zone_ok=false;
            }
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

/* Called from the main loop. Settles the local UTC offset about once a second. */
/* Current UTC second, and the milliseconds into it. A negative trim can put utc_tick a little
   after 'now', so this works in signed arithmetic. */
static uint32_t utc_now(const Trail* t, uint32_t now, uint16_t* ms) {
    int32_t el=(int32_t)(now-t->utc_tick);
    int32_t sec=el/1000;
    int32_t rem=el%1000;
    if(rem<0) {
        rem+=1000;
        sec-=1;
    }
    if(ms) *ms=(uint16_t)rem;
    return t->utc_epoch+(uint32_t)sec;
}

static void tz_update(Trail* t) {
    if(!t->utc_valid) { t->tz_known=false; return; }
    uint32_t now=furi_get_tick();
    if(t->tz_last_tick!=0 && (uint32_t)(now-t->tz_last_tick)<1000) return;
    t->tz_last_tick=now;
    if(!t->settings.tz_auto) {
        t->tz_offset_min=(int16_t)(t->settings.tz_half_hours*30);
        t->tz_known=true;
        t->tz_solar=false;
        return;
    }
    if(!t->pos_known) { t->tz_known=false; return; }
    uint32_t utc=utc_now(t,now,NULL);
    int off=0;
    if(tz_lookup(&t->tz,t->lat,t->lon,utc,&off)) {
        t->tz_offset_min=(int16_t)off;
        t->tz_known=true;
        t->tz_solar=false;
    } else {
        /* no tz.bin: rough solar time, one hour per 15 degrees of longitude */
        t->tz_offset_min=(int16_t)((int)roundf((float)t->lon/15.0f)*60);
        t->tz_known=true;
        t->tz_solar=true;
    }
}

/* Keep the Flipper's tick clock locked to the GPS second. The RMC sentence arrives a little
   after the second begins and by a slightly different delay each time, so the start of each
   second is averaged over many sentences (1/8 weight) instead of trusting a single one. */
static void utc_sync(Trail* t, uint32_t e, uint32_t start_tick) {
    uint32_t now=furi_get_tick();
    if(t->utc_valid) {
        int32_t step=(int32_t)(e-t->utc_epoch); /* whole seconds since the last sync */
        if(step==0) return; /* a second RMC (another constellation) for the same second */
        if(step>0 && step<3600) {
            uint32_t proj=t->utc_tick+(uint32_t)step*1000u;
            int32_t err=(int32_t)(start_tick-proj);
            if(err>-1500 && err<1500) {
                t->utc_tick=proj+(uint32_t)(err/8);
                t->utc_epoch=e;
                t->utc_sync_tick=now;
                return;
            }
        }
    }
    t->utc_epoch=e;
    t->utc_tick=start_tick;
    t->utc_valid=true;
    t->utc_sync_tick=now;
}

/* Set the Flipper's own clock to GPS local time, so the Clock app, file dates and other apps
   agree with it. The Flipper has no time zone: it simply holds whatever local time we write.
   The write is timed to the start of a GPS second so the clock's seconds line up, and it is
   repeated if the clock drifts by 2 s or more (e.g. at a daylight-saving change) or every 30 min. */
static void trail_clock_sync(Trail* t) {
    if(!t->settings.sync_clock || !t->utc_valid || !t->tz_known || !t->fix) {
        t->clock_pending=false;
        return;
    }
    uint32_t now=furi_get_tick();
    if((uint32_t)(now-t->utc_sync_tick)>3000u) { /* only trust fresh GPS time */
        t->clock_pending=false;
        return;
    }
    int64_t offset=(int64_t)t->tz_offset_min*60;
    if(!t->clock_pending) {
        if(t->clock_check_tick!=0 && (uint32_t)(now-t->clock_check_tick)<5000u) return;
        t->clock_check_tick=now;
        int64_t local=(int64_t)utc_now(t,now,NULL)+offset;
        DateTime rtc;
        furi_hal_rtc_get_datetime(&rtc);
        int64_t diff=(int64_t)datetime_datetime_to_timestamp(&rtc)-local;
        bool due=!t->clock_synced || diff>1 || diff<-1 || (uint32_t)(now-t->clock_sync_tick)>1800000u;
        if(!due) return;
        t->clock_pending=true;
    }
    /* pending: write exactly when the next GPS second begins (we are polled every ~100 ms) */
    uint16_t into=0;
    uint32_t cur=utc_now(t,now,&into);
    uint32_t wait=1000u-into;
    if(wait>120u) return;
    uint32_t boundary=now+wait;
    int64_t target=(int64_t)(cur+1u)+offset;
    DateTime dt;
    datetime_timestamp_to_datetime((uint32_t)target,&dt);
    while((int32_t)(furi_get_tick()-boundary)<0) furi_delay_ms(1);
    furi_hal_rtc_set_datetime(&dt);
    t->clock_synced=true;
    t->clock_pending=false;
    t->clock_sync_tick=furi_get_tick();
}

static void append_live_point(Trail* t) {
    Point point={.lat=t->lat,.lon=t->lon,.alt=t->alt_smooth_ok ? t->alt_smooth : NAN};

    furi_mutex_acquire(t->mutex,FuriWaitForever);
    if(t->route_count>0) {
        Point last=t->route[t->route_count-1];
        if(dist_m(last.lat,last.lon,point.lat,point.lon)<0.75f) {
            furi_mutex_release(t->mutex);
            return;
        }
    }

    if(t->route_count>=300) {
        Point last=t->route[t->route_count-1];
        size_t out=1;
        for(size_t i=2;i+1<t->route_count;i+=2) {
            t->route[out++]=t->route[i];
        }
        t->route[out++]=last;
        t->route_count=out;
    }

    if(t->route_count<300) {
        t->route[t->route_count++]=point;
    }
    furi_mutex_release(t->mutex);
}

static bool coord(const char* value, const char* hemi, double* out) {
    if(!value || !*value || !hemi || !*hemi) return false;
    /* ddmm.mmmmm: whole degrees and minutes are split in double so no digits are lost */
    double raw=parse_double(value);
    double deg=floor(raw/(double)100.0);
    double result=deg+(raw-deg*(double)100.0)/(double)60.0;
    if(hemi[0]=='S' || hemi[0]=='W') result=-result;
    *out=result; return true;
}
static bool field(char* line, int wanted, char* out, size_t cap) {
    char* p=line; int index=0;
    while(index<wanted) { p=strchr(p, ','); if(!p) return false; p++; index++; }
    char* end=strchr(p, ','); if(!end) end=p+strlen(p);
    size_t n=(size_t)(end-p); if(n>=cap) n=cap-1;
    memcpy(out,p,n); out[n]='\0'; return true;
}
/* Convert NMEA RMC hhmmss.sss and ddmmyy into an ISO 8601 UTC timestamp. */
static bool parse_nmea_timestamp(const char* time, const char* date, char* out, size_t cap) {
    if(!time || !date || !out || cap < 21 || strlen(time) < 6 || strlen(date) < 6) return false;
    for(size_t i=0; i<6; i++) {
        if(time[i]<'0' || time[i]>'9' || date[i]<'0' || date[i]>'9') return false;
    }
    int hour=(time[0]-'0')*10+(time[1]-'0');
    int minute=(time[2]-'0')*10+(time[3]-'0');
    int second=(time[4]-'0')*10+(time[5]-'0');
    int day=(date[0]-'0')*10+(date[1]-'0');
    int month=(date[2]-'0')*10+(date[3]-'0');
    int year=(date[4]-'0')*10+(date[5]-'0');
    year += year >= 80 ? 1900 : 2000;
    if(hour>23 || minute>59 || second>60 || day<1 || day>31 || month<1 || month>12) return false;
    int n=snprintf(out,cap,"%04d-%02d-%02dT%02d:%02d:%02dZ",
                   year,month,day,hour,minute,second);
    return n>0 && (size_t)n<cap;
}

/* "YYYY-MM-DDTHH:MM:SSZ" -> UTC epoch seconds. Rejects implausible years (module RTC junk). */
static bool timestamp_to_epoch(const char* ts, uint32_t* epoch) {
    if(!ts || strlen(ts)<19) return false;
    static const uint8_t digit_pos[]={0,1,2,3,5,6,8,9,11,12,14,15,17,18};
    for(size_t i=0;i<sizeof(digit_pos);i++) {
        if(ts[digit_pos[i]]<'0' || ts[digit_pos[i]]>'9') return false;
    }
    DateTime dt;
    memset(&dt,0,sizeof(dt));
    dt.year=(uint16_t)((ts[0]-'0')*1000+(ts[1]-'0')*100+(ts[2]-'0')*10+(ts[3]-'0'));
    dt.month=(uint8_t)((ts[5]-'0')*10+(ts[6]-'0'));
    dt.day=(uint8_t)((ts[8]-'0')*10+(ts[9]-'0'));
    dt.hour=(uint8_t)((ts[11]-'0')*10+(ts[12]-'0'));
    dt.minute=(uint8_t)((ts[14]-'0')*10+(ts[15]-'0'));
    dt.second=(uint8_t)((ts[17]-'0')*10+(ts[18]-'0'));
    if(dt.second>59) dt.second=59; /* leap second */
    dt.weekday=1; /* only needed to pass validation */
    if(dt.year<2025 || dt.year>2099) return false;
    if(!datetime_validate_datetime(&dt)) return false;
    *epoch=datetime_datetime_to_timestamp(&dt);
    return true;
}

static bool gps_quality_good(const Trail* t) {
    /* Ignore a metric when the receiver hasn't provided it. */
    if(t->fix_quality >= 0 && t->fix_quality == 0) return false;
    if(t->satellites >= 0 && t->satellites < 4) return false;
    if(t->hdop >= 0.0f && t->hdop > 5.0f) return false;
    return true;
}

static void save_point(Trail* t) {
    if(t->mode!=ModeRecording || !t->fix) return;

    float step_m=0.0f;
    float tracked_segment_m=0.0f;

    /* Raw displacement between consecutive GPS samples. */
    if(t->have_sample_prev) {
        step_m=dist_m(t->sample_prev_lat,t->sample_prev_lon,t->lat,t->lon);
    }
    t->sample_prev_lat=t->lat;
    t->sample_prev_lon=t->lon;
    t->have_sample_prev=true;

    /*
     * Require a sustained speed pattern, not a single GPS speed spike:
     * fast movement: 3 consecutive readings >= 0.8 m/s;
     * slow movement: 5 consecutive readings from 0.25 to < 0.8 m/s.
     */
    if(t->speed_valid && t->speed_mps >= 0.8f) {
        if(t->fast_speed_samples < 3) t->fast_speed_samples++;
        t->slow_speed_samples=0;
    } else if(t->speed_valid && t->speed_mps >= 0.25f) {
        if(t->slow_speed_samples < 5) t->slow_speed_samples++;
        t->fast_speed_samples=0;
    } else {
        t->slow_speed_samples=0;
        t->fast_speed_samples=0;
    }

    if(!t->have_prev) {
        t->prev_lat=t->lat;
        t->prev_lon=t->lon;
        t->have_prev=true;
    } else {
        float d=dist_m(t->prev_lat,t->prev_lon,t->lat,t->lon);

        if(d>=250.0f) {
            /* Reject a large jump and reset the tracking anchor. */
            t->prev_lat=t->lat;
            t->prev_lon=t->lon;
            t->slow_speed_samples=0;
            t->fast_speed_samples=0;
        } else if(!gps_quality_good(t)) {
            /* Poor fix quality: don't accumulate drift across bad samples. */
            t->prev_lat=t->lat;
            t->prev_lon=t->lon;
            t->slow_speed_samples=0;
            t->fast_speed_samples=0;
        } else if(t->speed_valid && t->speed_mps >= 0.8f) {
            /* Count fast movement only after three consecutive fast readings. */
            if(t->fast_speed_samples >= 3 && d>=2.0f) {
                t->distance+=(double)d;
                tracked_segment_m=d;
                t->prev_lat=t->lat;
                t->prev_lon=t->lon;
                t->fast_speed_samples=0;
            }
        } else if(t->speed_valid && t->speed_mps >= 0.25f) {
            /* Count slow movement only after five consecutive slow readings. */
            if(t->slow_speed_samples >= 5 && d>=2.0f) {
                t->distance+=(double)d;
                tracked_segment_m=d;
                t->prev_lat=t->lat;
                t->prev_lon=t->lon;
                t->slow_speed_samples=0;
            }
        } else {
            /*
             * Treat near-zero or unavailable speed as stationary for this sample.
             * Re-anchor instead of letting low-speed GPS drift accumulate.
             */
            t->prev_lat=t->lat;
            t->prev_lon=t->lon;
        }
    }

    uint32_t now_ms=furi_get_tick();
    if(t->route_count==0 || tracked_segment_m>0.0f) t->last_move_tick=now_ms;
    uint32_t idle_ms=(uint32_t)idle_s[t->settings.idle_idx]*1000u;
    t->idle=(idle_ms>0 && (uint32_t)(now_ms-t->last_move_tick)>=idle_ms);

    if(t->route_count==0 || tracked_segment_m>0.0f) {
        append_live_point(t);
        /* saved routes: store the segment to this accepted, quality-filtered point */
        rt_record_point(t);
    }
    if(t->idle) return; /* standing still: no new rows until you move again */
    t->points++;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* file=storage_file_alloc(storage);
    if(storage_file_open(file,t->route_path,FSAM_WRITE,FSOM_OPEN_APPEND)) {
        char row[208];
        char altitude_out[20]="NA";
        float speed_out=t->speed_valid ? t->speed_mps : -1.0f;
        if(t->altitude_valid)
            snprintf(altitude_out,sizeof(altitude_out),"%.2f",(double)t->altitude_m);
        snprintf(row,sizeof(row),
                 "%.7f,%.7f,%s,%.2f,%d,%.2f,%d,%.2f,%.2f,%.2f,%s\r\n",
                 t->lat,t->lon,t->timestamp,
                 (double)speed_out,t->satellites,(double)t->hdop,t->fix_quality,
                 (double)step_m,(double)tracked_segment_m,t->distance,altitude_out);
        storage_file_write(file,row,strlen(row));
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}
/* Course over ground from the RMC sentence, smoothed; only trusted while actually moving. */
static void heading_update(Trail* t, char* line) {
    if(!t->speed_valid || t->speed_mps<0.8f) return;
    char txt[16]={0};
    if(!field(line,8,txt,sizeof(txt)) || txt[0]<'0' || txt[0]>'9') return;
    float a=rad(parse_float(txt));
    if(!t->hdg_valid) {
        t->hdg_sin=sinf(a);
        t->hdg_cos=cosf(a);
        t->hdg_valid=true;
    } else {
        const float k=0.35f;
        t->hdg_sin=t->hdg_sin*(1.0f-k)+sinf(a)*k;
        t->hdg_cos=t->hdg_cos*(1.0f-k)+cosf(a)*k;
    }
}

static int hex_val(char c) {
    if(c>='0' && c<='9') return c-'0';
    if(c>='A' && c<='F') return c-'A'+10;
    if(c>='a' && c<='f') return c-'a'+10;
    return -1;
}
/* Verify the *XX checksum so dropped/garbled bytes never reach the parser. */
static bool nmea_checksum_ok(const char* line) {
    if(!line || line[0]!='$') return false;
    uint8_t sum=0;
    const char* p=line+1;
    while(*p && *p!='*') { sum^=(uint8_t)*p; p++; }
    if(*p!='*') return false;
    int hi=hex_val(p[1]);
    if(hi<0) return false;
    int lo=hex_val(p[2]);
    if(lo<0) return false;
    return sum==(uint8_t)((hi<<4)|lo);
}
static void parse_line(Trail* t, char* line) {
    if(line[0]!='$' || !nmea_checksum_ok(line)) return;
    char sentence[8]={0}; char* comma=strchr(line, ','); if(!comma) return;
    size_t n=(size_t)(comma-line); if(n>sizeof(sentence)-1) n=sizeof(sentence)-1;
    memcpy(sentence,line,n);
    char valid[8]={0}, la[20]={0}, ns[4]={0}, lo[20]={0}, ew[4]={0}, quality[8]={0};
    if(strstr(sentence,"RMC")) {
        char utc_time[20]={0}, utc_date[12]={0};
        bool time_ok=field(line,1,utc_time,sizeof(utc_time)) &&
                     field(line,9,utc_date,sizeof(utc_date)) &&
                     parse_nmea_timestamp(utc_time,utc_date,t->timestamp,sizeof(t->timestamp));
        if(!time_ok) {
            strlcpy(t->timestamp,"UNKNOWN",sizeof(t->timestamp));
        } else {
            uint32_t e;
            if(timestamp_to_epoch(t->timestamp,&e)) {
                utc_sync(t,e,t->line_start_tick-(uint32_t)(int32_t)t->settings.trim_ms);
            }
        }
        char speed_knots[16]={0};
        t->speed_valid=field(line,7,speed_knots,sizeof(speed_knots)) &&
                       speed_knots[0]>='0' && speed_knots[0]<='9';
        t->speed_mps=t->speed_valid ? parse_float(speed_knots)*0.514444f : 0.0f;
        if(field(line,2,valid,sizeof(valid)) && field(line,3,la,sizeof(la)) &&
           field(line,4,ns,sizeof(ns)) && field(line,5,lo,sizeof(lo)) && field(line,6,ew,sizeof(ew))) {
            double lat,lon; bool ok=coord(la,ns,&lat)&&coord(lo,ew,&lon);
            t->fix=(valid[0]=='A')&&ok;
            if(t->fix) { t->lat=lat; t->lon=lon; t->pos_known=true; heading_update(t,line); tilestore_follow(&t->tiles,lat,lon); save_point(t); }
        }
    } else if(strstr(sentence,"GGA")) {
        char sats[8]={0}, hdop_text[16]={0};
        if(field(line,7,sats,sizeof(sats)) && sats[0]>='0' && sats[0]<='9')
            t->satellites=atoi(sats);
        else
            t->satellites=-1;
        if(field(line,8,hdop_text,sizeof(hdop_text)) &&
           hdop_text[0]>='0' && hdop_text[0]<='9')
            t->hdop=parse_float(hdop_text);
        else
            t->hdop=-1.0f;
        if(field(line,6,quality,sizeof(quality)) &&
           quality[0]>='0' && quality[0]<='9')
            t->fix_quality=atoi(quality);
        else
            t->fix_quality=-1;

        /* GGA field 9 is altitude; field 10 should identify metres. */
        char altitude_text[16]={0}, altitude_unit[4]={0};
        bool altitude_ok =
            field(line,9,altitude_text,sizeof(altitude_text)) &&
            field(line,10,altitude_unit,sizeof(altitude_unit)) &&
            ( (altitude_text[0]>='0' && altitude_text[0]<='9') ||
              altitude_text[0]=='-' || altitude_text[0]=='+' ) &&
            altitude_unit[0]=='M';
        t->altitude_valid=altitude_ok;
        if(altitude_ok) {
            float a=parse_float(altitude_text);
            t->altitude_m=a; /* raw value still feeds stats + CSV */
            t->alt_smooth=t->alt_smooth_ok ? (0.8f*t->alt_smooth+0.2f*a) : a;
            t->alt_smooth_ok=true;
        } else {
            t->alt_smooth_ok=false;
        }

        if(field(line,2,la,sizeof(la)) && field(line,3,ns,sizeof(ns)) &&
           field(line,4,lo,sizeof(lo)) && field(line,5,ew,sizeof(ew)) && field(line,6,quality,sizeof(quality))) {
            double lat,lon; bool ok=coord(la,ns,&lat)&&coord(lo,ew,&lon);
            t->fix=(atoi(quality)>0)&&ok;
            if(t->fix) { t->lat=lat; t->lon=lon; }
        }
    }
}
static void rx_callback(FuriHalSerialHandle* serial, FuriHalSerialRxEvent event, void* context) {
    Trail* t=context;
    if(event!=FuriHalSerialRxEventData) return;
    /* Keep interrupt callback short: queue raw bytes; parsing and SD writes happen in app thread. */
    uint8_t c=furi_hal_serial_async_rx(serial);
    if(c=='\n') { /* stamp first, so the app thread never sees a line without its time */
        uint8_t h=t->nl_head;
        uint8_t nh=(uint8_t)((h+1)&31);
        if(nh!=t->nl_tail) {
            t->nl_tick[h]=furi_get_tick();
            t->nl_head=nh;
        }
    }
    RxByte byte={.byte=c};
    furi_message_queue_put(t->rx_queue,&byte,0);
}
static void map_local_xy(
    double lat, double lon, double ref_lat, double ref_lon,
    float* east, float* north) {
    *east=(float)((lon-ref_lon)*(double)111320.0)*cosf(rad((float)ref_lat));
    *north=(float)((lat-ref_lat)*(double)111320.0);
}
