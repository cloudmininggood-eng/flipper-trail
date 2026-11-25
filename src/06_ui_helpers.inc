/* flipper_trail module: ui helpers. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---------------- Shared UI helpers ---------------- */
static const char* tab_name(TrailTab tab) {
    switch(tab) {
    case TrailTabRoute: return "Route";
    case TrailTabMap: return "Map";
    case TrailTabStats: return "Stats";
    case TrailTabTime: return "Time";
    case TrailTabElev: return "Elev";
    default: return "";
    }
}

static float alt_conv(float m, bool imp) { return imp ? m*3.28084f : m; }
static const char* alt_unit(bool imp) { return imp ? "ft" : "m"; }

static void fmt_dist(char* out, size_t cap, float m, bool imp) {
    if(imp) {
        float ft=m*3.28084f;
        if(ft<1000.0f) snprintf(out,cap,"%.0fft",(double)ft);
        else snprintf(out,cap,"%.2fmi",(double)(m/1609.344f));
    } else {
        if(m<1000.0f) snprintf(out,cap,"%.0fm",(double)m);
        else snprintf(out,cap,"%.2fkm",(double)(m/1000.0f));
    }
}

static bool trail_on_map(const Trail* t) {
    if(t->settings_open) return false;
    if(t->mode==ModeHome) return t->home_page==HomePageMap;
    if(t->mode==ModeRecording) return t->active_tab==TrailTabMap;
    return false;
}

/* Settings open from the menu, stats and time screens only (Up is free on the map).
   The time screen opens straight at the time zone page. */
static bool trail_settings_allowed(const Trail* t) {
    if(t->mode==ModeHome)
        return t->home_page==HomePageMenu || t->home_page==HomePageStats || t->home_page==HomePageTime;
    if(t->mode==ModeRecording) return t->active_tab==TrailTabStats || t->active_tab==TrailTabTime;
    return false;
}

static bool trail_on_time(const Trail* t) {
    if(t->settings_open) return false;
    if(t->mode==ModeHome) return t->home_page==HomePageTime;
    if(t->mode==ModeRecording) return t->active_tab==TrailTabTime;
    return false;
}

static bool trail_on_route(const Trail* t) {
    if(t->settings_open) return false;
    if(t->mode==ModeHome) return t->home_page==HomePageRoute;
    if(t->mode==ModeRecording) return t->active_tab==TrailTabRoute;
    return false;
}

/* ---- Map held upright (64x128): sizes come from the canvas, so the helpers work in both ---- */
#define TALL_FOOTER_H 10
static bool canvas_is_tall(const Canvas* canvas) { return canvas_width(canvas)<canvas_height(canvas); }
static int body_bottom(const Canvas* canvas) {
    return canvas_is_tall(canvas) ? (int)canvas_height(canvas)-TALL_FOOTER_H-2 : UI_BODY_BOTTOM;
}
static int body_right(const Canvas* canvas) { return (int)canvas_width(canvas)-1; }
static int map_cx(const Canvas* canvas) { return (int)canvas_width(canvas)/2; }
static int map_cy(const Canvas* canvas) { return (UI_BODY_TOP+body_bottom(canvas))/2; }

/* The screen orientation the current page wants: only the map can be held upright. */
static ViewPortOrientation trail_orientation(const Trail* t) {
    if(!trail_on_map(t)) return ViewPortOrientationHorizontal;
    if(t->settings.map_screen==MapScreenTall) return ViewPortOrientationVertical;
    if(t->settings.map_screen==MapScreenTallFlip) return ViewPortOrientationVerticalFlip;
    return ViewPortOrientationHorizontal;
}

/* Upright header: state on the left; the Back hint only when there is room for it. */
static void draw_header_tall(Canvas* canvas, Trail* t) {
    canvas_set_font(canvas,FontSecondary);
    const int W=(int)canvas_width(canvas);
    bool moving=(t->map_free && trail_on_map(t));
    const char* right=moving ? "BK:DONE" : ((t->mode==ModeHome) ? "BK:REC" : "BK:STOP");
    int right_x=W-2-(int)canvas_string_width(canvas,right);
    char left[24];
    int left_x=2;
    if(t->mode==ModeRecording) {
        if(t->idle) {
            canvas_draw_box(canvas,3,2,2,7);
            canvas_draw_box(canvas,7,2,2,7);
        }
        else if((furi_get_tick()/500)%2==0) canvas_draw_disc(canvas,5,5,2);
        else canvas_draw_circle(canvas,5,5,2);
        left_x=11;
    }
    if(moving) {
        snprintf(left,sizeof(left),"MOVE");
    } else if(t->mode==ModeRecording) {
        char d[16];
        fmt_dist(d,sizeof(d),(float)t->distance,t->settings.imperial);
        snprintf(left,sizeof(left),"%s %s",t->idle ? "IDLE" : "REC",d);
        if(left_x+(int)canvas_string_width(canvas,left) > W-2)
            snprintf(left,sizeof(left),"%s",t->idle ? "IDLE" : "REC");
    } else {
        snprintf(left,sizeof(left),"MAP");
    }
    canvas_draw_str(canvas,left_x,8,left);
    if(left_x+(int)canvas_string_width(canvas,left)+4<=right_x) canvas_draw_str(canvas,right_x,8,right);
    canvas_draw_line(canvas,0,10,W-1,10);
}

/* Header: state on the left, what the Back button does on the right. */
static void draw_header(Canvas* canvas, Trail* t, const char* home_title) {
    if(canvas_is_tall(canvas)) {
        draw_header_tall(canvas,t);
        return;
    }
    canvas_set_font(canvas,FontSecondary);
    bool moving=(t->map_free && trail_on_map(t));
    const char* right=moving ? "BACK:DONE" : ((t->mode==ModeHome) ? "BACK:REC" : "BACK:STOP");
    int right_x=126-(int)canvas_string_width(canvas,right);
    char left[32];
    int left_x=2;

    if(t->mode==ModeRecording) {
        if(t->idle) {
            canvas_draw_box(canvas,3,2,2,7); /* pause bars: not saving while you stand still */
            canvas_draw_box(canvas,7,2,2,7);
        }
        else if((furi_get_tick()/500)%2==0) canvas_draw_disc(canvas,5,5,2);
        else canvas_draw_circle(canvas,5,5,2);
        left_x=11;
    }
    if(moving) {
        snprintf(left,sizeof(left),"MOVE hold OK:pin");
        if(left_x+(int)canvas_string_width(canvas,left) > right_x-6)
            snprintf(left,sizeof(left),"MOVE");
    } else if(t->mode==ModeRecording) {
        char d[16];
        fmt_dist(d,sizeof(d),(float)t->distance,t->settings.imperial);
        snprintf(left,sizeof(left),"%s %s",t->idle ? "IDLE" : "REC",d);
        /* Drop the distance if it would crowd the right-hand hint. */
        if(left_x+(int)canvas_string_width(canvas,left) > right_x-6)
            snprintf(left,sizeof(left),"%s",t->idle ? "IDLE" : "REC");
    } else {
        snprintf(left,sizeof(left),"%s",home_title);
    }
    canvas_draw_str(canvas,left_x,8,left);
    canvas_draw_str(canvas,right_x,8,right);
    canvas_draw_line(canvas,0,10,127,10);
}

/* Centre footer hint (same look as the native button hints). */
static void draw_hint_box(Canvas* canvas, const char* label) {
    int w=(int)canvas_string_width(canvas,label)+6;
    int x=64-w/2;
    canvas_draw_box(canvas,x,52,(size_t)w,12);
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_str(canvas,x+3,61,label);
    canvas_set_color(canvas,ColorBlack);
}

/* Label of the OK button on the Route page: what OK does on the focused row. */
static const char* route_center_label(const Trail* t) {
    if(t->routing) return "Stop";
    int sel=(t->wp_count>0) ? (t->route_sel<t->wp_count ? (int)t->route_sel : (int)t->wp_count-1) : -1;
    switch(t->route_row) {
    case 0: return "Next";
    case 1: return (sel>=0 && t->wp[sel].id==t->settings.dest_id) ? "Clear" : "Go";
    case 2: return "Rename";
    case 3: return "Radius";
    case 4: return "Hold:Del";
    case 5: return "Pin";
    case 6: return "Start";
    default: return "Change";
    }
}

/* Footer: each hint matches exactly what that key does. */
/* Total climb and descent along the recorded track (changes under 3 m are ignored as GPS noise). */
static void elev_gain_loss(const Trail* t, float* up, float* down) {
    *up=0.0f;
    *down=0.0f;
    bool have_ref=false;
    float ref=0.0f;
    for(size_t i=0;i<t->route_count;i++) {
        float a=t->route[i].alt;
        if(isnan(a)) continue;
        if(!have_ref) {
            ref=a;
            have_ref=true;
            continue;
        }
        float d=a-ref;
        if(d>=3.0f) {
            *up+=d;
            ref=a;
        } else if(d<=-3.0f) {
            *down-=d;
            ref=a;
        }
    }
}

/* Plain readouts in footer slots without a button, so the bottom row is never left empty. */
static void draw_footer_info(Canvas* canvas, Trail* t, bool has_l, bool has_r) {
    char b[40];
    bool imp=t->settings.imperial;
    canvas_set_font(canvas,FontSecondary);
    if(trail_on_route(t)) {
        if(has_l) return;
        if(t->routing) {
            if(t->fix && t->speed_valid)
                snprintf(b,sizeof(b),"%.1f%s",(double)(imp ? t->speed_mps*2.23694f : t->speed_mps),imp ? "mph" : "m/s");
            else
                snprintf(b,sizeof(b),"--%s",imp ? "mph" : "m/s");
        } else {
            snprintf(b,sizeof(b),"%u pts",(unsigned)t->wp_count);
        }
        canvas_draw_str(canvas,2,61,b);
    } else if(trail_on_time(t)) {
        if(has_r) return;
        if(!t->settings.sync_clock) snprintf(b,sizeof(b),"Clk off");
        else if(t->clock_synced) snprintf(b,sizeof(b),"Clk %um",(unsigned)((furi_get_tick()-t->clock_sync_tick)/60000u));
        else snprintf(b,sizeof(b),"Clk wait");
        canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,b),61,b);
    } else if(t->mode==ModeRecording && t->active_tab==TrailTabElev && !has_r && t->route_count>=2) {
        float up,down;
        elev_gain_loss(t,&up,&down);
        snprintf(b,sizeof(b),"+%.0f%s  -%.0f%s",(double)alt_conv(up,imp),alt_unit(imp),(double)alt_conv(down,imp),alt_unit(imp));
        canvas_draw_str(canvas,126-(int)canvas_string_width(canvas,b),61,b);
    }
}

/* Upright footer: one slim row of hints (the wide button shapes do not fit in 64 px). */
static void draw_footer_tall(Canvas* canvas, const char* l, const char* r, bool free_move) {
    const int W=(int)canvas_width(canvas);
    const int H=(int)canvas_height(canvas);
    canvas_set_font(canvas,FontSecondary);
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_box(canvas,0,H-TALL_FOOTER_H,(size_t)W,TALL_FOOTER_H);
    canvas_set_color(canvas,ColorBlack);
    canvas_draw_line(canvas,0,H-TALL_FOOTER_H,W-1,H-TALL_FOOTER_H);
    if(free_move) {
        canvas_draw_str_aligned(canvas,W/2,H-2,AlignCenter,AlignBottom,"OK:done");
        return;
    }
    char bl[16]={0}, br[16]={0};
    if(l) snprintf(bl,sizeof(bl),"<%s",l);
    if(r) snprintf(br,sizeof(br),"%s>",r);
    if((int)canvas_string_width(canvas,bl)+(int)canvas_string_width(canvas,br)>W-4) { /* no room for the arrows */
        if(l) snprintf(bl,sizeof(bl),"%s",l);
        if(r) snprintf(br,sizeof(br),"%s",r);
    }
    if(l) canvas_draw_str(canvas,1,H-2,bl);
    if(r) canvas_draw_str(canvas,W-1-(int)canvas_string_width(canvas,br),H-2,br);
}

static void draw_footer(Canvas* canvas, Trail* t) {
    const char* l=NULL;
    const char* r=NULL;
    const char* c=NULL;
    bool on_map=trail_on_map(t);
    bool on_route=trail_on_route(t);
    if(on_map && t->map_free) {
        l="Move";
        r="Move";
        c="Done";
    } else if(t->mode==ModeHome) {
        if(t->home_page==HomePageRoute) { r="Map"; }
        else if(t->home_page==HomePageMap) { l="Route"; r="Menu"; }
        else if(t->home_page==HomePageMenu) { l="Map"; r="Stats"; }
        else if(t->home_page==HomePageStats) { l="Menu"; r="Time"; }
        else { l="Stats"; }
    } else if(t->mode==ModeRecording) {
        if(t->active_tab>TrailTabRoute) l=tab_name((TrailTab)(t->active_tab-1));
        if(t->active_tab<(TrailTab)(TrailTabCount-1)) r=tab_name((TrailTab)(t->active_tab+1));
    } else {
        return;
    }
    if(canvas_is_tall(canvas)) {
        draw_footer_tall(canvas,l,r,on_map && t->map_free);
        return;
    }
    canvas_set_font(canvas,FontSecondary);
    if(on_route) c=route_center_label(t);
    if(l) elements_button_left(canvas,l);
    if(r) elements_button_right(canvas,r);
    if(c) elements_button_center(canvas,c);
    else if(on_map && t->settings.free_move) draw_hint_box(canvas,"OK:Move");
    else if(trail_on_time(t)) draw_hint_box(canvas,"Up:Zone");
    else if(trail_settings_allowed(t)) draw_hint_box(canvas,"Up:Set");
    draw_footer_info(canvas,t,l!=NULL,r!=NULL);
}
