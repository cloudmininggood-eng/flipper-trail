/* flipper_trail module: route page. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---------------- Route page ---------------- */
#define ROUTE_ROWS 8

static int route_sel_idx(Trail* t) {
    if(t->wp_count==0) return -1;
    if(t->route_sel>=t->wp_count) t->route_sel=(uint8_t)(t->wp_count-1);
    return (int)t->route_sel;
}

static void route_row_text(Trail* t, int row, char* out, size_t cap) {
    int sel=route_sel_idx(t);
    const Waypoint* w=(sel>=0) ? &t->wp[sel] : NULL;
    bool imp=t->settings.imperial;
    switch(row) {
    case 0:
        if(!w) {
            snprintf(out,cap,"Point: none");
        } else if(t->pos_known) {
            char d[16];
            wp_dist_str(t,w,d,sizeof(d));
            snprintf(out,cap,"%s: %s %s%s",w->radius_m>0 ? "Area" : "Point",w->name,d,
                     w->id==t->settings.dest_id ? " *" : "");
        } else {
            snprintf(out,cap,"%s: %s%s",w->radius_m>0 ? "Area" : "Point",w->name,
                     w->id==t->settings.dest_id ? " *" : "");
        }
        break;
    case 1:
        snprintf(out,cap,"Go to %s%s",(w && w->radius_m>0) ? "area" : "point",
                 (w && w->id==t->settings.dest_id) ? " *" : "");
        break;
    case 2: snprintf(out,cap,"Rename point"); break;
    case 3:
        if(!w || w->radius_m==0) {
            snprintf(out,cap,"Radius: none (point)");
        } else {
            char d[16];
            fmt_dist(d,sizeof(d),(float)w->radius_m,imp);
            snprintf(out,cap,"Radius: %s",d);
        }
        break;
    case 4: snprintf(out,cap,"Delete point"); break;
    case 5: snprintf(out,cap,"Pin here"); break;
    case 6: snprintf(out,cap,"Start routing"); break;
    default:
        if(imp) snprintf(out,cap,"Link gap: %.0f ft",(double)(link_m[t->settings.link_idx]*3.28084f));
        else snprintf(out,cap,"Link gap: %.0f m",(double)link_m[t->settings.link_idx]);
        break;
    }
}

static void draw_toast_bar(Canvas* canvas, const char* text) {
    canvas_set_font(canvas,FontSecondary);
    canvas_draw_box(canvas,0,38,128,12);
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_str_aligned(canvas,64,47,AlignCenter,AlignBottom,text);
    canvas_set_color(canvas,ColorBlack);
}

static void draw_route_list(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    draw_header(canvas,t,"ROUTE");
    canvas_set_font(canvas,FontSecondary);
    int row=t->route_row;
    if(row>=ROUTE_ROWS) row=ROUTE_ROWS-1;
    const int vis=3, pitch=12;
    int top=row-1;
    if(top<0) top=0;
    if(top>ROUTE_ROWS-vis) top=ROUTE_ROWS-vis;
    for(int i=0;i<vis;i++) {
        int r=top+i;
        if(r>=ROUTE_ROWS) break;
        char b[32];
        route_row_text(t,r,b,sizeof(b));
        int y=UI_BODY_TOP+i*pitch; /* top of this row */
        if(r==row) {
            canvas_draw_box(canvas,0,y,125,pitch);
            canvas_set_color(canvas,ColorWhite);
            canvas_draw_str(canvas,3,y+9,b);
            canvas_set_color(canvas,ColorBlack);
        } else {
            canvas_draw_str(canvas,3,y+9,b);
        }
    }
    /* scroll bar: shows where these rows sit in the list */
    int track=vis*pitch;
    int thumb=track*vis/ROUTE_ROWS;
    int thumb_y=UI_BODY_TOP+(track-thumb)*top/(ROUTE_ROWS-vis);
    canvas_draw_line(canvas,127,UI_BODY_TOP,127,UI_BODY_TOP+track-1);
    canvas_draw_box(canvas,126,thumb_y,2,thumb);
    if(t->toast[0] && (int32_t)(furi_get_tick()-t->toast_until)<0) draw_toast_bar(canvas,t->toast);
}

/* Routing view: only the way from where you are to the destination. */
static void draw_route_nav(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    draw_header(canvas,t,"ROUTING");
    canvas_set_font(canvas,FontSecondary);
    bool imp=t->settings.imperial;
    int wi=wp_index_by_id(t,t->settings.dest_id);
    if(wi<0) {
        draw_empty_state(canvas,"No destination","Stop, then pick a point");
        return;
    }
    if(!t->pos_known) {
        draw_empty_state(canvas,"Waiting for GPS","Routing needs your position");
        return;
    }
    const Waypoint* w=&t->wp[wi];

    MapDraw m;
    memset(&m,0,sizeof(m));
    m.canvas=canvas;
    if(!global_xy(t->lat,t->lon,&m.ref_gx,&m.ref_gy)) return;
    m.unit_h_m=111320.0f/(CELLS_PER_DEG*(float)RT_UNITS);
    m.unit_w_m=m.unit_h_m*cosf(rad(t->lat));
    m.rot_c=1.0f;
    m.rot_s=0.0f;

    float e2,n2; /* destination relative to you */
    map_local_xy(w->lat,w->lon,t->lat,t->lon,&e2,&n2);
    bool have_path=(t->nav_state==1 && t->nav_n>=2);

    float fh=0.0f;
    bool rotate=map_facing(t,&fh);
    if(rotate) {
        /* turned view: you near the bottom, the way ahead going up; the scale only depends on
           how far the farthest point is, so turning does not make the zoom pulse */
        m.rot_c=cosf(fh);
        m.rot_s=sinf(fh);
        float reach=sqrtf(e2*e2+n2*n2);
        if(have_path) {
            for(uint16_t i=0;i<t->nav_n;i++) {
                float e=(float)(t->nav_path[i].x-m.ref_gx)*m.unit_w_m;
                float n=(float)(t->nav_path[i].y-m.ref_gy)*m.unit_h_m;
                float d=sqrtf(e*e+n*n);
                if(d>reach) reach=d;
            }
        }
        if(reach<25.0f) reach=25.0f;
        m.scale=30.0f/reach;
        m.center_e=0.0f;
        m.center_n=14.0f/m.scale;
    } else {
        ext_add(&m.ext,0.0f,0.0f);
        ext_add(&m.ext,e2,n2);
        if(have_path) {
            for(uint16_t i=0;i<t->nav_n;i++) {
                ext_add(&m.ext,(float)(t->nav_path[i].x-m.ref_gx)*m.unit_w_m,
                               (float)(t->nav_path[i].y-m.ref_gy)*m.unit_h_m);
            }
        }
        m.center_e=(m.ext.min_e+m.ext.max_e)/2.0f;
        m.center_n=(m.ext.min_n+m.ext.max_n)/2.0f;
        float span_e=m.ext.max_e-m.ext.min_e;
        float span_n=m.ext.max_n-m.ext.min_n;
        if(span_e<20.0f) span_e=20.0f;
        if(span_n<20.0f) span_n=20.0f;
        span_e*=1.25f;
        span_n*=1.25f;
        float sx=124.0f/span_e;
        float sy=(float)(UI_BODY_BOTTOM-UI_BODY_TOP-4)/span_n;
        m.scale=sx<sy ? sx : sy;
    }

    float dx0,dy0,dx1,dy1;
    map_xy(&m,0.0f,0.0f,&dx0,&dy0);   /* you */
    map_xy(&m,e2,n2,&dx1,&dy1);       /* destination */
    if(have_path) {
        for(int pass=0;pass<3;pass++) {
            float ox=(pass==1) ? 1.0f : 0.0f;
            float oy=(pass==2) ? 1.0f : 0.0f;
            for(uint16_t i=1;i<t->nav_n;i++) {
                float ax,ay,bx,by;
                map_xy_g(&m,t->nav_path[i-1].x,t->nav_path[i-1].y,&ax,&ay);
                map_xy_g(&m,t->nav_path[i].x,t->nav_path[i].y,&bx,&by);
                draw_clipped_line(canvas,ax+ox,ay+oy,bx+ox,by+oy);
            }
        }
    } else {
        draw_dotted_line(canvas,dx0,dy0,dx1,dy1);
    }
    if(w->radius_m>0) draw_area_circle(canvas,dx1,dy1,(float)w->radius_m*m.scale);
    draw_marker(canvas,dx1,dy1,MarkerDest);
    draw_marker(canvas,dx0,dy0,t->fix ? MarkerYou : MarkerLast);
    draw_scale_bar(canvas,m.scale,imp);
    if(rotate) draw_compass(canvas,m.rot_c,m.rot_s);

    int ly=21;
    char b[32], d[16];
    float dm=sqrtf(e2*e2+n2*n2);
    if(w->radius_m>0) { /* an area: distance to its edge */
        dm-=(float)w->radius_m;
        if(dm<0.0f) dm=0.0f;
    }
    if(dm<=0.0f && w->radius_m>0) {
        snprintf(b,sizeof(b),"in %s",w->name);
    } else if(dm<20.0f) {
        snprintf(b,sizeof(b),"%s here",w->name);
    } else if(t->nav_state==1) {
        fmt_dist(d,sizeof(d),t->nav_route_m,imp);
        snprintf(b,sizeof(b),"%s %s",w->name,d);
    } else if(t->nav_state==2) {
        snprintf(b,sizeof(b),"%s no route",w->name);
    } else if(t->nav_state==3) {
        snprintf(b,sizeof(b),"%s too far",w->name);
    } else {
        snprintf(b,sizeof(b),"%s ...",w->name);
    }
    push_label(canvas,&ly,b);
    if(dm>=20.0f) {
        if(have_path) {
            /* heading for the next point along the way (skip points right under you) */
            uint16_t k=1;
            float e=(float)(t->nav_path[k].x-m.ref_gx)*m.unit_w_m;
            float n=(float)(t->nav_path[k].y-m.ref_gy)*m.unit_h_m;
            if(sqrtf(e*e+n*n)<8.0f && t->nav_n>2) {
                k=2;
                e=(float)(t->nav_path[k].x-m.ref_gx)*m.unit_w_m;
                n=(float)(t->nav_path[k].y-m.ref_gy)*m.unit_h_m;
            }
            fmt_dist(d,sizeof(d),sqrtf(e*e+n*n),imp);
            snprintf(b,sizeof(b),"next %s %s",d,compass8(e,n));
        } else {
            fmt_dist(d,sizeof(d),dm,imp);
            snprintf(b,sizeof(b),"%s %s",d,compass8(e2,n2));
        }
        push_label(canvas,&ly,b);
    }
    if(!t->fix) push_label(canvas,&ly,"NO FIX");
}

static void draw_dashboard(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    draw_header(canvas,t,"STATS");
    canvas_set_font(canvas,FontSecondary);
    bool imp=t->settings.imperial;

    char b[64], d[16];
    if(!t->fix) {
        snprintf(b,sizeof(b),"GPS NO FIX");
    } else if(t->satellites>=0 && t->hdop>=0.0f && t->fix_quality>=0) {
        snprintf(b,sizeof(b),"GPS FIX S%d H%.1f Q%d",
                 t->satellites,(double)t->hdop,t->fix_quality);
    } else {
        snprintf(b,sizeof(b),"GPS FIX WAIT");
    }
    canvas_draw_str(canvas,2,20,b);
    {
        char clk[12];
        fmt_clock(t,clk,sizeof(clk));
        canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,clk),20,clk);
    }

    if(t->fix)
        snprintf(b,sizeof(b),"%c%.5f %c%.5f",
                 t->lat>=0 ? 'N' : 'S',fabs(t->lat),
                 t->lon>=0 ? 'E' : 'W',fabs(t->lon));
    else
        snprintf(b,sizeof(b),"Position --");
    canvas_draw_str(canvas,2,29,b);

    char alt[20], spd[20];
    if(t->fix && t->altitude_valid)
        snprintf(alt,sizeof(alt),"Alt %.1f%s",(double)alt_conv(t->altitude_m,imp),alt_unit(imp));
    else
        snprintf(alt,sizeof(alt),"Alt --");
    if(t->fix && t->speed_valid)
        snprintf(spd,sizeof(spd),"%.1f%s",
                 (double)(imp ? t->speed_mps*2.23694f : t->speed_mps),imp ? "mph" : "m/s");
    else
        snprintf(spd,sizeof(spd),"--%s",imp ? "mph" : "m/s");
    snprintf(b,sizeof(b),"%s  %s",alt,spd);
    canvas_draw_str(canvas,2,38,b);

    fmt_dist(d,sizeof(d),(float)t->distance,imp);
    snprintf(b,sizeof(b),"%s %lupt map+%lu",d,(unsigned long)t->points,(unsigned long)t->tiles.new_pts);
    canvas_draw_str(canvas,2,47,b);
}

static void draw_home_menu(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    canvas_set_font(canvas,FontPrimary);
    canvas_draw_str(canvas,2,11,"FLIPPER TRAIL");
    canvas_set_font(canvas,FontSecondary);

    char b[48];
    if(!t->fix)
        snprintf(b,sizeof(b),"%s  GPS NO FIX",t->status);
    else if(t->satellites>=0)
        snprintf(b,sizeof(b),"%s  GPS FIX S%d",t->status,t->satellites);
    else
        snprintf(b,sizeof(b),"%s  GPS FIX",t->status);
    canvas_draw_str(canvas,2,21,b);
    canvas_draw_str(canvas,2,30,"BACK: start recording");
    canvas_draw_str(canvas,2,39,"HOLD BACK: exit");
    canvas_draw_str(canvas,2,48,"UP: settings");
    {
        char clk[12];
        fmt_clock(t,clk,sizeof(clk));
        canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,clk),11,clk);
    }
}

/* Local clock to the second, from GPS time. */
static void draw_time(Canvas* canvas, Trail* t) {
    static const char* const wdays[7]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    static const char* const months[12]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    canvas_clear(canvas);
    draw_header(canvas,t,"TIME");
    canvas_set_font(canvas,FontSecondary);
    if(!t->utc_valid) {
        canvas_set_font(canvas,FontPrimary);
        canvas_draw_str_aligned(canvas,64,30,AlignCenter,AlignBottom,"No GPS time yet");
        canvas_set_font(canvas,FontSecondary);
        canvas_draw_str_aligned(canvas,64,42,AlignCenter,AlignBottom,"Needs satellites in view");
        return;
    }
    uint32_t now=furi_get_tick();
    uint32_t utc=utc_now(t,now,NULL);
    bool zone=t->tz_known;
    int off_min=zone ? (int)t->tz_offset_min : 0;
    int64_t local=(int64_t)utc+(int64_t)off_min*60;
    DateTime dt, du;
    datetime_timestamp_to_datetime((uint32_t)local,&dt);
    datetime_timestamp_to_datetime(utc,&du);
    char b[40];
    snprintf(b,sizeof(b),"%02u:%02u:%02u",(unsigned)dt.hour,(unsigned)dt.minute,(unsigned)dt.second);
    canvas_set_font(canvas,FontBigNumbers);
    canvas_draw_str_aligned(canvas,64,28,AlignCenter,AlignBottom,b); /* digits fill y14..28 */
    canvas_set_font(canvas,FontSecondary);
    uint32_t days=(uint32_t)(local/86400);
    unsigned mo=dt.month>=1 && dt.month<=12 ? dt.month-1u : 0u;
    snprintf(b,sizeof(b),"%s %u %s %u",wdays[(days+4u)%7u],(unsigned)dt.day,months[mo],(unsigned)dt.year);
    canvas_draw_str_aligned(canvas,64,38,AlignCenter,AlignBottom,b);
    snprintf(b,sizeof(b),"UTC %02u:%02u:%02u",(unsigned)du.hour,(unsigned)du.minute,(unsigned)du.second);
    canvas_draw_str(canvas,2,48,b);
    char r[16];
    uint32_t age=(now-t->utc_sync_tick)/1000u;
    if(age>5) {
        snprintf(r,sizeof(r),"stale %lus",(unsigned long)(age>99 ? 99 : age));
    } else if(!zone) {
        snprintf(r,sizeof(r),"no zone");
    } else {
        char o[12];
        fmt_offset(o,sizeof(o),off_min);
        snprintf(r,sizeof(r),"%s%s",o,t->tz_solar ? "*" : "");
    }
    canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,r),48,r);
}

static void draw_elevation(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    draw_header(canvas,t,"ELEVATION");
    canvas_set_font(canvas,FontSecondary);
    bool imp=t->settings.imperial;

    float lo=0,hi=0,total=0; bool have=false;
    for(size_t i=0;i<t->route_count;i++) {
        if(i) total+=dist_m(t->route[i-1].lat,t->route[i-1].lon,
                            t->route[i].lat,t->route[i].lon);
        float a=t->route[i].alt;
        if(isnan(a)) continue;
        if(!have) { lo=hi=a; have=true; }
        else { if(a<lo) lo=a; if(a>hi) hi=a; }
    }
    if(!have || total<1.0f) {
        draw_empty_state(canvas,"No elevation yet","Fills in as you move");
        return;
    }
    float vmin=lo, vmax=hi; /* real range for the label */
    if(hi-lo<10.0f) { float mid=(lo+hi)/2.0f; lo=mid-5.0f; hi=mid+5.0f; }

    char b[48], d[16];
    /* while recording, the distance the Stats page counts (the graph's own track is thinned out) */
    float label_m=(t->mode==ModeRecording && t->distance>(double)0.0f) ? (float)t->distance : total;
    fmt_dist(d,sizeof(d),label_m,imp);
    snprintf(b,sizeof(b),"Elev %.0f-%.0f%s %s",
             (double)alt_conv(vmin,imp),(double)alt_conv(vmax,imp),alt_unit(imp),d);
    canvas_draw_str(canvas,2,19,b);
    canvas_draw_frame(canvas,0,21,128,29);

    float run=0; bool have_prev=false; int px=0,py=0;
    for(size_t i=0;i<t->route_count;i++) {
        if(i) run+=dist_m(t->route[i-1].lat,t->route[i-1].lon,
                          t->route[i].lat,t->route[i].lon);
        float a=t->route[i].alt;
        if(isnan(a)) continue;
        int x=2+(int)(run/total*123.0f);
        int y=47-(int)((a-lo)/(hi-lo)*23.0f);
        if(have_prev) canvas_draw_line(canvas,px,py,x,y);
        px=x; py=y; have_prev=true;
    }
    if(have_prev) canvas_draw_disc(canvas,px,py,2); /* latest point */
}

/* Short, so the centre button never touches the neighbour names beside it. */
static const char* settings_center_label(uint8_t page) {
    switch(page) {
    case SettingsOptimize: return "Run";
    case SettingsTimeZone: return "Mode";
    case SettingsTrim: return "-50";
    case SettingsZoom: return "Wider";
    default: return "Next";
    }
}

static void draw_settings(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    canvas_set_font(canvas,FontSecondary);
    char b[40];
    snprintf(b,sizeof(b),"SETUP %u/%u",(unsigned)t->settings_page+1u,(unsigned)SettingsPageCount);
    canvas_draw_str(canvas,2,8,b);
    const char* right="DOWN:BACK";
    canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,right),8,right);
    canvas_draw_line(canvas,0,10,127,10);

    canvas_set_font(canvas,FontPrimary);
    canvas_draw_str(canvas,2,24,settings_names[t->settings_page]);
    if(t->settings_page==SettingsTimeZone || t->settings_page==SettingsTrim) {
        char clk[24];
        canvas_set_font(canvas,FontSecondary);
        if(t->settings_page==SettingsTrim) fmt_clock_tenths(t,clk,sizeof(clk));
        else fmt_clock(t,clk,sizeof(clk));
        canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,clk),24,clk);
        canvas_set_font(canvas,FontPrimary);
    }
    char v[24];
    settings_value_text(t,v,sizeof(v));
    snprintf(b,sizeof(b),"[ %s ]",v);
    canvas_draw_str_aligned(canvas,64,37,AlignCenter,AlignBottom,b);
    canvas_set_font(canvas,FontSecondary);
    const char* hint_text=settings_hints[t->settings_page];
    char hint[48];
    char ctr[16];
    const char* ctr_text=settings_center_label(t->settings_page);
    if(t->settings_page==SettingsTrim) { /* the labels follow the chosen step */
        int s=trim_step_ms(&t->settings);
        snprintf(hint,sizeof(hint),"Up:+%d  OK:-%d  hold:0",s,s);
        snprintf(ctr,sizeof(ctr),"-%d",s);
        hint_text=hint;
        ctr_text=ctr;
    }
    canvas_draw_str_aligned(canvas,64,48,AlignCenter,AlignBottom,hint_text);

    if(t->settings_page>0) elements_button_left(canvas,settings_short[t->settings_page-1]);
    elements_button_center(canvas,ctr_text);
    if(t->settings_page<SettingsPageCount-1)
        elements_button_right(canvas,settings_short[t->settings_page+1]);
}

static void draw_inner(Canvas* canvas, Trail* t) {
    if(t->settings_open) {
        draw_settings(canvas,t);
        return;
    }
    if(trail_on_route(t)) {
        if(t->routing) draw_route_nav(canvas,t);
        else draw_route_list(canvas,t);
    } else if(t->mode==ModeHome) {
        if(t->home_page==HomePageMenu) draw_home_menu(canvas,t);
        else if(t->home_page==HomePageMap) draw_live_map(canvas,t);
        else if(t->home_page==HomePageTime) draw_time(canvas,t);
        else draw_dashboard(canvas,t);
    } else if(t->active_tab==TrailTabMap) {
        draw_live_map(canvas,t);
    } else if(t->active_tab==TrailTabTime) {
        draw_time(canvas,t);
    } else if(t->active_tab==TrailTabElev) {
        draw_elevation(canvas,t);
    } else {
        draw_dashboard(canvas,t);
    }
    draw_footer(canvas,t);
}

static void draw(Canvas* canvas, void* context) {
    Trail* t=context;
    furi_mutex_acquire(t->mutex,FuriWaitForever);
    /* the screen is mid-turn (state changed, orientation not yet): skip, the next update redraws */
    if(canvas_is_tall(canvas)!=(trail_orientation(t)!=ViewPortOrientationHorizontal)) {
        furi_mutex_release(t->mutex);
        return;
    }
    draw_inner(canvas,t);
    furi_mutex_release(t->mutex);
}
static void input_cb(InputEvent* event, void* context) {
    FuriMessageQueue* q=context; UiEvent ev={.event=*event}; furi_message_queue_put(q,&ev,0); /* never block the GUI thread */
}
static void begin_recording(Trail* t) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    storage_common_mkdir(storage,ROUTES_DIR);
    DateTime dt;
    furi_hal_rtc_get_datetime(&dt);
    File* f=storage_file_alloc(storage);
    bool opened=false;
    for(int attempt=0;attempt<2 && !opened;attempt++) {
        if(attempt==0) {
            snprintf(t->route_path,sizeof(t->route_path),
                     ROUTES_DIR "/walk_%04u%02u%02u_%02u%02u%02u.csv",
                     (unsigned)dt.year,(unsigned)dt.month,(unsigned)dt.day,
                     (unsigned)dt.hour,(unsigned)dt.minute,(unsigned)dt.second);
        } else {
            snprintf(t->route_path,sizeof(t->route_path),
                     ROUTES_DIR "/walk_%04u%02u%02u_%02u%02u%02u_%lu.csv",
                     (unsigned)dt.year,(unsigned)dt.month,(unsigned)dt.day,
                     (unsigned)dt.hour,(unsigned)dt.minute,(unsigned)dt.second,
                     (unsigned long)furi_get_tick());
        }
        /* CREATE_NEW: never overwrite an earlier recording */
        opened=storage_file_open(f,t->route_path,FSAM_WRITE,FSOM_CREATE_NEW);
    }
    if(opened) {
        const char* header="latitude,longitude,timestamp_utc,speed_mps,satellites,hdop,fix_quality,step_m,tracked_segment_m,total_distance_m,altitude_m\r\n";
        storage_file_write(f,header,strlen(header));
        t->mode=ModeRecording; t->tiles.new_pts=0; t->rt_have_last=false; t->distance=0; t->points=0;
        t->have_prev=false; t->have_sample_prev=false;
        t->slow_speed_samples=0; t->fast_speed_samples=0;
        t->last_move_tick=furi_get_tick(); t->idle=false;
        furi_mutex_acquire(t->mutex,FuriWaitForever); t->route_count=0; furi_mutex_release(t->mutex);
        t->active_tab=TrailTabMap; t->home_page=HomePageMenu;
        strlcpy(t->status,"RECORDING",sizeof(t->status));
    } else {
        strlcpy(t->status,"SD ERROR",sizeof(t->status));
    }
    storage_file_close(f); storage_file_free(f); furi_record_close(RECORD_STORAGE);
}
static bool read_line(File* file, char* out, size_t cap) {
    if(!out || cap < 1) return false;
    size_t len=0;
    bool got=false;
    char c;
    while(storage_file_read(file, &c, 1)==1) {
        got=true;
        if(c=='\n') break;
        if(c=='\r') continue;
        if(len+1<cap) out[len++]=c;
    }
    out[len]='\0';
    return got;
}

static void load_latest_route(Trail* t) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    /* Minimal v0.1: the user selects a route path by editing this path after recording.
       Route browser will be added after hardware/SDK validation. */
    if(storage_file_open(f,t->route_path,FSAM_READ,FSOM_OPEN_EXISTING)) {
        char row[208];
        furi_mutex_acquire(t->mutex,FuriWaitForever);
        t->route_count=0;
        while(t->route_count<300 && read_line(f,row,sizeof(row))) {
            if(row[0]=='l' || row[0]=='\0') continue;
            /* altitude_m is CSV field 10; read it before the row is split below */
            char alt_text[16]={0};
            float alt=NAN;
            if(field(row,10,alt_text,sizeof(alt_text)) &&
               ((alt_text[0]>='0' && alt_text[0]<='9') || alt_text[0]=='-' || alt_text[0]=='+'))
                alt=parse_float(alt_text);
            char* comma=strchr(row,','); if(!comma) continue; *comma++='\0';
            t->route[t->route_count].lat=parse_double(row); t->route[t->route_count].lon=parse_double(comma);
            t->route[t->route_count].alt=alt; t->route_count++;
        }
        t->target_index=t->route_count? t->route_count-1:0;
        furi_mutex_release(t->mutex);
    }
    storage_file_close(f); storage_file_free(f); furi_record_close(RECORD_STORAGE);
}
