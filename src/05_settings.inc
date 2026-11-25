/* flipper_trail module: settings. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---------------- Settings ---------------- */
#define SETTINGS_PATH "/ext/GPS/settings.bin"
#define SETTINGS_MAGIC "FTS"
#define SETTINGS_VERSION 12
#define TRIM_MAX_MS 3000
static const uint8_t trim_steps[]={1,5,10,50,100}; /* ms per tap; 50 is the default */
#define TRIM_STEP_COUNT ((int)(sizeof(trim_steps)/sizeof(trim_steps[0])))
#define TRIM_STEP_DEFAULT 3
static int trim_step_ms(const TrailSettings* st) {
    return trim_steps[st->trim_step_idx<TRIM_STEP_COUNT ? st->trim_step_idx : TRIM_STEP_DEFAULT];
}
#define ZOOM_COUNT 13
#define ZOOM_DEFAULT 6 /* 200 m */
#define BAUD_COUNT 2

typedef struct {
    char magic[3];
    uint8_t version;
    TrailSettings s;
} SettingsFile;

static const float zoom_span_m[ZOOM_COUNT]={20.0f,30.0f,50.0f,75.0f,100.0f,150.0f,200.0f,300.0f,400.0f,600.0f,800.0f,1200.0f,1600.0f};
static const uint32_t baud_values[BAUD_COUNT]={9600,115200};
static const char* const settings_names[SettingsPageCount]={
    "Destination","Map view","Map facing","Map screen","Map layers","Map zoom","Map OK key","Move step",
    "Merge routes","Link gap","Clean up routes","Idle stop","Units","Time zone","Time trim","Trim step","Sync clock","GPS baud"};
static const char* const settings_short[SettingsPageCount]={
    "Dest","View","Face","Tall","Layer","Zoom","Key","Step","Merge","Link","Clean","Idle","Units","Time","Trim","Fine","Clock","Baud"};
static const char* const settings_hints[SettingsPageCount]={
    "OK:next  hold OK:delete","Fit=whole  Follow=you","Follow view & routing","Wide / held upright",
    "Saved = past routes","OK:wider  Up:closer","OK: look, hold: pin","Distance per tap",
    "Drop repeated paths","Max gap to join dots","Import CSVs, merge dups",
    "Stop saving when still",
    "Distance, alt, speed","OK:Auto/Man  Up:+30m","Up:+50  OK:-50  hold:0","Size of one trim tap","Set Flipper clock (GPS)","9600 std, 115200 Pro"};

static void fmt_offset(char* out, size_t cap, int minutes) {
    int a=minutes<0 ? -minutes : minutes;
    if(a%60) snprintf(out,cap,"UTC%c%d:%02d",minutes<0 ? '-' : '+',a/60,a%60);
    else snprintf(out,cap,"UTC%c%d",minutes<0 ? '-' : '+',a/60);
}

/* Local time as HH:MM (24 h), or --:-- until GPS has given us time and zone. */
static void fmt_clock_tenths(const Trail* t, char* out, size_t cap) {
    if(!t->utc_valid) {
        snprintf(out,cap,"--:--:--.---");
        return;
    }
    uint16_t ms=0;
    uint32_t utc=utc_now(t,furi_get_tick(),&ms);
    int64_t local=(int64_t)utc+(t->tz_known ? (int64_t)t->tz_offset_min*60 : 0);
    DateTime dt;
    datetime_timestamp_to_datetime((uint32_t)local,&dt);
    snprintf(out,cap,"%02u:%02u:%02u.%03u",(unsigned)dt.hour,(unsigned)dt.minute,(unsigned)dt.second,(unsigned)ms);
}

static void fmt_clock(const Trail* t, char* out, size_t cap) {
    if(!t->utc_valid || !t->tz_known) {
        snprintf(out,cap,"--:--");
        return;
    }
    uint32_t utc=utc_now(t,furi_get_tick(),NULL);
    int64_t local=(int64_t)utc+(int64_t)t->tz_offset_min*60;
    DateTime dt;
    datetime_timestamp_to_datetime((uint32_t)local,&dt);
    snprintf(out,cap,"%02u:%02u",(unsigned)dt.hour,(unsigned)dt.minute);
}

static void settings_defaults(TrailSettings* st) {
    st->map_view=MapViewFit;
    st->map_layer=MapLayerBoth;
    st->zoom=ZOOM_DEFAULT;
    st->free_move=1;
    st->merge_idx=2;
    st->move_idx=0;
    st->link_idx=1;
    st->dest_id=0;
    st->facing=FacingHeading;
    st->map_screen=MapScreenWide;
    st->idle_idx=2;
    st->sync_clock=1;
    st->trim_ms=0;
    st->trim_step_idx=TRIM_STEP_DEFAULT;
    st->imperial=0;
    st->baud_idx=0;
    st->tz_auto=1;
    st->tz_half_hours=0;
}

static void settings_load(TrailSettings* st) {
    settings_defaults(st);
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,SETTINGS_PATH,FSAM_READ,FSOM_OPEN_EXISTING)) {
        uint8_t raw[4+sizeof(TrailSettings)];
        size_t n=storage_file_read(f,raw,sizeof(raw));
        /* version 1 files only hold the first six fields; the rest keep their defaults */
        if(n>=4+6 && memcmp(raw,SETTINGS_MAGIC,3)==0 && raw[3]>=1 && raw[3]<=SETTINGS_VERSION) {
            TrailSettings tmp=*st;
            size_t body=n-4;
            if(body>sizeof(tmp)) body=sizeof(tmp);
            memcpy(&tmp,raw+4,body);
            /* older files had padding where map_screen now lives */
            if(raw[3]<10 || tmp.map_screen>=MapScreenCount) tmp.map_screen=MapScreenWide;
            if(tmp.trim_step_idx>=TRIM_STEP_COUNT) tmp.trim_step_idx=TRIM_STEP_DEFAULT;
            if(raw[3]<12) { /* zoom used to be 100/200/400 m only: keep the same widths */
                static const uint8_t old_zoom[3]={4,6,8};
                tmp.zoom=tmp.zoom<3 ? old_zoom[tmp.zoom] : ZOOM_DEFAULT;
            }
            if(tmp.map_view<MapViewCount && tmp.map_layer<MapLayerCount &&
               tmp.zoom<ZOOM_COUNT && tmp.baud_idx<BAUD_COUNT &&
               tmp.tz_half_hours>=-24 && tmp.tz_half_hours<=28 && tmp.merge_idx<MERGE_COUNT &&
               tmp.move_idx<MOVE_COUNT && tmp.link_idx<LINK_COUNT &&
               tmp.facing<FacingCount && tmp.idle_idx<IDLE_COUNT && tmp.sync_clock<=1 &&
               tmp.trim_ms>=-TRIM_MAX_MS && tmp.trim_ms<=TRIM_MAX_MS) {
                /* older files used this byte for something else: default to on */
                tmp.free_move=(raw[3]>=4) ? (tmp.free_move ? 1 : 0) : 1;
                tmp.imperial=tmp.imperial ? 1 : 0;
                tmp.tz_auto=tmp.tz_auto ? 1 : 0;
                *st=tmp;
            }
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

static void settings_save(const TrailSettings* st) {
    SettingsFile sf={.magic={'F','T','S'},.version=SETTINGS_VERSION,.s=*st};
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,SETTINGS_PATH,FSAM_WRITE,FSOM_CREATE_ALWAYS)) {
        storage_file_write(f,&sf,sizeof(sf));
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

/* Change the trim and move the running clock by the difference straight away. + shows the clock
   later, so the second is taken to have started earlier. Wraps around at +-3 s. */
static void trim_set(Trail* t, int v) {
    if(v>TRIM_MAX_MS) v=-TRIM_MAX_MS;
    if(v<-TRIM_MAX_MS) v=TRIM_MAX_MS;
    int delta=v-(int)t->settings.trim_ms;
    t->settings.trim_ms=(int16_t)v;
    if(t->utc_valid) t->utc_tick-=(uint32_t)(int32_t)delta;
    t->clock_synced=false; /* the Flipper's own clock follows within a few seconds */
    t->clock_pending=false;
    t->clock_check_tick=0;
    settings_save(&t->settings);
}

/* Destination page, OK: none -> first saved location -> next ... -> none. */
/* Set (or clear, id 0) the destination. The Route page's selected point follows it, so the Route
   page, the Destination setting and the map always name the same point. */
static void dest_set(Trail* t, uint8_t id) {
    t->settings.dest_id=id;
    int i=wp_index_by_id(t,id);
    if(i>=0) t->route_sel=(uint8_t)i;
    if(id==0) t->routing=false;
    t->nav_dirty=true;
    settings_save(&t->settings);
}

static void settings_cycle_dest(Trail* t) {
    int next=wp_index_by_id(t,t->settings.dest_id)+1; /* current index, or -1 for none */
    dest_set(t,(next>=(int)t->wp_count) ? 0 : t->wp[next].id);
}

/* Destination page, hold OK: delete the selected saved location. */
static void settings_delete_dest(Trail* t) {
    TrailSettings* st=&t->settings;
    int idx=wp_index_by_id(t,st->dest_id);
    if(idx<0) return;
    wp_delete(t,idx);
    st->dest_id=0;
    t->nav_dirty=true;
    settings_save(st);
}

/* OK on the settings page: cycle the shown setting, apply it, and save. */
static void settings_change(Trail* t, FuriHalSerialHandle* serial) {
    TrailSettings* st=&t->settings;
    switch(t->settings_page) {
    case SettingsMapView: st->map_view=(uint8_t)((st->map_view+1)%MapViewCount); break;
    case SettingsMapFacing: st->facing=(uint8_t)((st->facing+1)%FacingCount); break;
    case SettingsMapScreen: st->map_screen=(uint8_t)((st->map_screen+1)%MapScreenCount); break;
    case SettingsMapLayer: st->map_layer=(uint8_t)((st->map_layer+1)%MapLayerCount); break;
    case SettingsZoom: st->zoom=(uint8_t)((st->zoom+1)%ZOOM_COUNT); break;
    case SettingsDestination: settings_cycle_dest(t); break;
    case SettingsFreeMove: st->free_move=st->free_move ? 0 : 1; break;
    case SettingsMoveStep: st->move_idx=(uint8_t)((st->move_idx+1)%MOVE_COUNT); break;
    case SettingsMerge: st->merge_idx=(uint8_t)((st->merge_idx+1)%MERGE_COUNT); break;
    case SettingsIdle: st->idle_idx=(uint8_t)((st->idle_idx+1)%IDLE_COUNT); break;
    case SettingsTrim: trim_set(t,(int)st->trim_ms-trim_step_ms(st)); break;
    case SettingsTrimStep: st->trim_step_idx=(uint8_t)((st->trim_step_idx+1)%TRIM_STEP_COUNT); break;
    case SettingsClock:
        st->sync_clock=st->sync_clock ? 0 : 1;
        t->clock_synced=false;
        t->clock_pending=false;
        t->clock_check_tick=0;
        break;
    case SettingsLink:
        st->link_idx=(uint8_t)((st->link_idx+1)%LINK_COUNT);
        t->nav_dirty=true;
        break;
    case SettingsOptimize:
        if(t->mode==ModeRecording) {
            strlcpy(t->opt_text,"Stop recording first",sizeof(t->opt_text));
        } else if(t->opt_state==0) {
            strlcpy(t->opt_text,"Working...",sizeof(t->opt_text));
            t->opt_state=1;
        }
        break;
    case SettingsUnits: st->imperial=st->imperial ? 0 : 1; break;
    case SettingsTimeZone:
        if(st->tz_auto) {
            /* switch to manual, starting from the offset auto had found */
            int hh=t->tz_known ? (int)roundf((float)t->tz_offset_min/30.0f) : 0;
            if(hh<-24) hh=-24;
            if(hh>28) hh=28;
            st->tz_half_hours=(int8_t)hh;
            st->tz_auto=0;
        } else {
            st->tz_auto=1;
        }
        t->tz_last_tick=0;
        tz_update(t);
        break;
    case SettingsBaud:
        st->baud_idx=(uint8_t)((st->baud_idx+1)%BAUD_COUNT);
        furi_hal_serial_set_br(serial,baud_values[st->baud_idx]);
        t->line_len=0;
        break;
    default: break;
    }
    settings_save(st);
}

/* Up on the settings page: only the time zone uses it (+30 min, hold to repeat). */
static void settings_nudge(Trail* t) {
    if(t->settings_page==SettingsZoom) { /* OK goes wider, Up goes closer */
        t->settings.zoom=(uint8_t)((t->settings.zoom+ZOOM_COUNT-1)%ZOOM_COUNT);
        settings_save(&t->settings);
        return;
    }
    if(t->settings_page==SettingsTrim) {
        trim_set(t,(int)t->settings.trim_ms+trim_step_ms(&t->settings));
        return;
    }
    if(t->settings_page!=SettingsTimeZone) return;
    TrailSettings* st=&t->settings;
    int hh;
    if(st->tz_auto) {
        hh=(t->tz_known ? (int)roundf((float)t->tz_offset_min/30.0f) : 0)+1;
        st->tz_auto=0;
    } else {
        hh=st->tz_half_hours+1;
    }
    if(hh>28) hh=-24;
    if(hh<-24) hh=-24;
    st->tz_half_hours=(int8_t)hh;
    settings_save(st);
    t->tz_last_tick=0;
    tz_update(t);
}

static void fmt_dist(char* out, size_t cap, float m, bool imp);

/* Distance to a saved location, worded the same on every page: for an area it is the distance
   to its edge, and "inside" once you are in it. */
static bool wp_dist_str(const Trail* t, const Waypoint* w, char* out, size_t cap) {
    out[0]='\0';
    if(!w || !t->pos_known) return false;
    float e,n;
    map_local_xy(w->lat,w->lon,t->lat,t->lon,&e,&n);
    float dd=sqrtf(e*e+n*n);
    if(w->radius_m>0) {
        dd-=(float)w->radius_m;
        if(dd<=0.0f) {
            snprintf(out,cap,"inside");
            return true;
        }
    }
    fmt_dist(out,cap,dd,t->settings.imperial);
    return true;
}

static void settings_value_text(const Trail* t, char* out, size_t cap) {
    const TrailSettings* st=&t->settings;
    static const char* const layers[MapLayerCount]={"This trip","Saved","Both"};
    switch(t->settings_page) {
    case SettingsMapView:
        snprintf(out,cap,"%s",st->map_view==MapViewFollow ? "Follow me" : "Fit route");
        break;
    case SettingsMapFacing: {
        static const char* const faces[FacingCount]={"North up","Heading up","Route up"};
        snprintf(out,cap,"%s",faces[st->facing]);
        break;
    }
    case SettingsMapScreen: {
        static const char* const screens[MapScreenCount]={"Wide","Tall","Tall flip"};
        snprintf(out,cap,"%s",screens[st->map_screen]);
        break;
    }
    case SettingsMapLayer: snprintf(out,cap,"%s",layers[st->map_layer]); break;
    case SettingsZoom: {
        float w=zoom_span_m[st->zoom];
        if(st->imperial) {
            float ft=w*3.28084f;
            if(ft>=5280.0f) snprintf(out,cap,"%.1f mi wide",(double)(ft/5280.0f));
            else snprintf(out,cap,"%.0f ft wide",(double)ft);
        } else if(w>=1000.0f) {
            snprintf(out,cap,"%.1f km wide",(double)(w/1000.0f));
        } else {
            snprintf(out,cap,"%.0f m wide",(double)w);
        }
        break;
    }
    case SettingsDestination: {
        int wi=wp_index_by_id(t,st->dest_id);
        if(t->wp_count==0) {
            snprintf(out,cap,"No pins yet");
        } else if(wi<0) {
            snprintf(out,cap,"None");
        } else if(t->pos_known) {
            char d[16];
            wp_dist_str(t,&t->wp[wi],d,sizeof(d));
            snprintf(out,cap,"%s %s",t->wp[wi].name,d);
        } else {
            snprintf(out,cap,"%s",t->wp[wi].name);
        }
        break;
    }
    case SettingsFreeMove: snprintf(out,cap,"%s",st->free_move ? "On" : "Off"); break;
    case SettingsMoveStep:
        if(st->move_idx==0) snprintf(out,cap,"Auto (20 px)");
        else if(st->imperial) snprintf(out,cap,"%.0f ft",(double)(move_m[st->move_idx]*3.28084f));
        else snprintf(out,cap,"%.0f m",(double)move_m[st->move_idx]);
        break;
    case SettingsLink:
        if(st->imperial) snprintf(out,cap,"%.0f ft",(double)(link_m[st->link_idx]*3.28084f));
        else snprintf(out,cap,"%.0f m",(double)link_m[st->link_idx]);
        break;
    case SettingsMerge:
        if(st->merge_idx==0) snprintf(out,cap,"Off");
        else if(st->imperial) snprintf(out,cap,"within %.0f ft",(double)(merge_m[st->merge_idx]*3.28084f));
        else snprintf(out,cap,"within %.0f m",(double)merge_m[st->merge_idx]);
        break;
    case SettingsOptimize: snprintf(out,cap,"%s",t->opt_text[0] ? t->opt_text : "Press OK"); break;
    case SettingsTrim:
        if(st->trim_ms==0) snprintf(out,cap,"0 ms");
        else snprintf(out,cap,"%+d ms",(int)st->trim_ms);
        break;
    case SettingsTrimStep: snprintf(out,cap,"%d ms",trim_step_ms(st)); break;
    case SettingsClock:
        if(!st->sync_clock) snprintf(out,cap,"Off");
        else if(t->clock_synced) {
            unsigned m=(unsigned)((furi_get_tick()-t->clock_sync_tick)/60000u);
            snprintf(out,cap,"On, set %um ago",m);
        } else snprintf(out,cap,"On, waiting for fix");
        break;
    case SettingsIdle:
        if(st->idle_idx==0) snprintf(out,cap,"Off");
        else snprintf(out,cap,"after %u s",(unsigned)idle_s[st->idle_idx]);
        break;
    case SettingsUnits: snprintf(out,cap,"%s",st->imperial ? "Imperial" : "Metric"); break;
    case SettingsTimeZone: {
        char o[16];
        if(st->tz_auto) {
            if(!t->tz_known) {
                snprintf(out,cap,"Auto (waiting)");
            } else {
                fmt_offset(o,sizeof(o),t->tz_offset_min);
                snprintf(out,cap,"Auto%s %s",t->tz_solar ? "*" : "",o);
            }
        } else {
            fmt_offset(o,sizeof(o),st->tz_half_hours*30);
            snprintf(out,cap,"%s",o);
        }
        break;
    }
    case SettingsBaud: snprintf(out,cap,"%lu",(unsigned long)baud_values[st->baud_idx]); break;
    default: out[0]='\0'; break;
    }
}
