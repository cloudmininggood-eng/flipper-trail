/* flipper_trail module: map view. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---------------- Map drawing ---------------- */
/* centre of the map body; a variable named canvas must be in scope */
#define MAP_CX map_cx(canvas)
#define MAP_CY map_cy(canvas)

typedef struct { float min_e, max_e, min_n, max_n; bool have; } Ext;

static void ext_add(Ext* x, float e, float n) {
    if(!x->have) {
        x->min_e=x->max_e=e;
        x->min_n=x->max_n=n;
        x->have=true;
    } else {
        if(e<x->min_e) x->min_e=e;
        if(e>x->max_e) x->max_e=e;
        if(n<x->min_n) x->min_n=n;
        if(n>x->max_n) x->max_n=n;
    }
}

typedef struct {
    Canvas* canvas;
    int32_t ref_gx, ref_gy;    /* reference position, global units */
    float unit_w_m, unit_h_m;  /* metres per unit (east-west / north-south) at that latitude */
    float center_e, center_n;  /* view centre in the screen frame (after turning), metres */
    float scale;               /* pixels per metre */
    float rot_c, rot_s;        /* cos/sin of the facing angle; (1,0) = north up */
    Ext ext;
    float ac_x[WP_MAX], ac_y[WP_MAX], ac_r[WP_MAX]; /* areas on screen: centre and radius, pixels */
    uint8_t ac_n;
} MapDraw;

/* World metres (east/north of the reference) -> screen pixels, turned so the facing is up. */
static void map_xy(const MapDraw* m, float e, float n, float* px, float* py) {
    float re=e*m->rot_c-n*m->rot_s;
    float rn=e*m->rot_s+n*m->rot_c;
    *px=(float)map_cx(m->canvas)+(re-m->center_e)*m->scale;
    *py=(float)map_cy(m->canvas)-(rn-m->center_n)*m->scale;
}

static void map_xy_g(const MapDraw* m, int32_t gx, int32_t gy, float* px, float* py) {
    map_xy(m,(float)(gx-m->ref_gx)*m->unit_w_m,(float)(gy-m->ref_gy)*m->unit_h_m,px,py);
}

/* Clip to the map body, then draw. */
static bool clip_to_body(const Canvas* canvas, float* x0, float* y0, float* x1, float* y1) {
    return lb_clip(x0,y0,x1,y1,0.0f,(float)UI_BODY_TOP,(float)body_right(canvas),(float)body_bottom(canvas));
}

static void draw_clipped_line(Canvas* canvas, float x0, float y0, float x1, float y1) {
    if(!clip_to_body(canvas,&x0,&y0,&x1,&y1)) return;
    canvas_draw_line(canvas,(int)roundf(x0),(int)roundf(y0),(int)roundf(x1),(int)roundf(y1));
}

/* Dotted line (every second pixel): saved routes, so this trip's solid line stays distinct. */
static void draw_dotted_line(Canvas* canvas, float fx0, float fy0, float fx1, float fy1) {
    if(!clip_to_body(canvas,&fx0,&fy0,&fx1,&fy1)) return;
    int x0=(int)roundf(fx0), y0=(int)roundf(fy0);
    int x1=(int)roundf(fx1), y1=(int)roundf(fy1);
    int dx=abs(x1-x0), sx=x0<x1 ? 1 : -1;
    int dy=-abs(y1-y0), sy=y0<y1 ? 1 : -1;
    int err=dx+dy;
    int step=0;
    for(;;) {
        if(!(step&1)) canvas_draw_dot(canvas,x0,y0);
        if(x0==x1 && y0==y1) break;
        int e2=2*err;
        if(e2>=dy) { err+=dy; x0+=sx; }
        if(e2<=dx) { err+=dx; y0+=sy; }
        step++;
    }
}

static void map_areas_setup(const Trail* t, MapDraw* m, double ref_lat, double ref_lon) {
    m->ac_n=0;
    for(uint8_t i=0;i<t->wp_count && m->ac_n<WP_MAX;i++) {
        if(t->wp[i].radius_m==0) continue;
        float e,n,px,py;
        map_local_xy(t->wp[i].lat,t->wp[i].lon,ref_lat,ref_lon,&e,&n);
        map_xy(m,e,n,&px,&py);
        m->ac_x[m->ac_n]=px;
        m->ac_y[m->ac_n]=py;
        m->ac_r[m->ac_n]=(float)t->wp[i].radius_m*m->scale;
        m->ac_n++;
    }
}

static bool map_px_in_area(const MapDraw* m, float px, float py) {
    for(uint8_t k=0;k<m->ac_n;k++) {
        float dx=px-m->ac_x[k], dy=py-m->ac_y[k];
        if(dx*dx+dy*dy<m->ac_r[k]*m->ac_r[k]) return true;
    }
    return false;
}

/* A line, minus the parts that fall inside an area (route dots inside areas are not shown). */
static void draw_line_vis(const MapDraw* m, float ax, float ay, float bx, float by, bool dotted) {
    float lo[24], hi[24];
    int np=1;
    lo[0]=0.0f;
    hi[0]=1.0f;
    float dx=bx-ax, dy=by-ay;
    float a=dx*dx+dy*dy;
    for(uint8_t k=0;k<m->ac_n && np>0 && np<20;k++) {
        float fx=ax-m->ac_x[k], fy=ay-m->ac_y[k];
        float r2=m->ac_r[k]*m->ac_r[k];
        float c=fx*fx+fy*fy-r2;
        float s1,s2;
        if(a<0.000001f) {
            if(c<0.0f) np=0; /* a single point inside */
            continue;
        }
        float b=fx*dx+fy*dy;
        float disc=b*b-a*c;
        if(disc<=0.0f) continue;
        float sq=sqrtf(disc);
        s1=(-b-sq)/a;
        s2=(-b+sq)/a;
        if(s2<=0.0f || s1>=1.0f) continue;
        float nlo[24], nhi[24];
        int nn=0;
        for(int i=0;i<np;i++) {
            if(s2<=lo[i] || s1>=hi[i]) {
                nlo[nn]=lo[i];
                nhi[nn]=hi[i];
                nn++;
            } else {
                if(s1>lo[i]) { nlo[nn]=lo[i]; nhi[nn]=s1; nn++; }
                if(s2<hi[i]) { nlo[nn]=s2; nhi[nn]=hi[i]; nn++; }
            }
        }
        for(int i=0;i<nn;i++) { lo[i]=nlo[i]; hi[i]=nhi[i]; }
        np=nn;
    }
    for(int i=0;i<np;i++) {
        float x0=ax+dx*lo[i], y0=ay+dy*lo[i];
        float x1=ax+dx*hi[i], y1=ay+dy*hi[i];
        if(dotted) draw_dotted_line(m->canvas,x0,y0,x1,y1);
        else draw_clipped_line(m->canvas,x0,y0,x1,y1);
    }
}

static void draw_area_circle(Canvas* canvas, float cx, float cy, float r) {
    if(r<2.0f) return;
    int n=(int)(r*0.9f);
    if(n<16) n=16;
    if(n>64) n=64;
    float px=cx+r, py=cy;
    for(int i=1;i<=n;i++) {
        float ang=(float)i*(6.2831853f/(float)n);
        float x=cx+r*cosf(ang), y=cy+r*sinf(ang);
        draw_clipped_line(canvas,px,py,x,y);
        px=x;
        py=y;
    }
}

static void seg_extent_cb(void* ctx, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    MapDraw* m=ctx;
    ext_add(&m->ext,(float)(x0-m->ref_gx)*m->unit_w_m,(float)(y0-m->ref_gy)*m->unit_h_m);
    ext_add(&m->ext,(float)(x1-m->ref_gx)*m->unit_w_m,(float)(y1-m->ref_gy)*m->unit_h_m);
}

static void seg_draw_cb(void* ctx, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    MapDraw* m=ctx;
    float ax,ay,bx,by;
    map_xy_g(m,x0,y0,&ax,&ay);
    map_xy_g(m,x1,y1,&bx,&by);
    draw_line_vis(m,ax,ay,bx,by,true);
}

typedef enum { MarkerStart, MarkerYou, MarkerLast, MarkerPin, MarkerDest } MarkerKind;

/* Markers get a white halo so they stay visible on top of lines.
   Start = hollow circle, you = filled dot, last known position = cross,
   saved location = small square, destination = square inside a ring. */
static void draw_marker(Canvas* canvas, float fx, float fy, MarkerKind kind) {
    int x=(int)roundf(fx);
    int y=(int)roundf(fy);
    int r=(kind==MarkerDest) ? 5 : 3;
    if(x<r || x>body_right(canvas)-r || y<UI_BODY_TOP+r || y>body_bottom(canvas)-r) return;
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_disc(canvas,x,y,(size_t)r);
    canvas_set_color(canvas,ColorBlack);
    if(kind==MarkerYou) {
        canvas_draw_disc(canvas,x,y,2);
    } else if(kind==MarkerStart) {
        canvas_draw_circle(canvas,x,y,2);
    } else if(kind==MarkerPin) {
        canvas_draw_box(canvas,x-1,y-1,3,3);
    } else if(kind==MarkerDest) {
        canvas_draw_circle(canvas,x,y,4);
        canvas_draw_box(canvas,x-1,y-1,3,3);
    } else {
        canvas_draw_line(canvas,x-2,y-2,x+2,y+2);
        canvas_draw_line(canvas,x-2,y+2,x+2,y-2);
    }
}

/* Right-aligned text on a white patch so it is readable over lines. */
static void draw_label_right(Canvas* canvas, int y_base, const char* text) {
    int w=(int)canvas_string_width(canvas,text);
    int x=(int)canvas_width(canvas)-2-w;
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_box(canvas,x-2,y_base-8,(size_t)(w+3),10);
    canvas_set_color(canvas,ColorBlack);
    canvas_draw_str(canvas,x,y_base,text);
}

static void draw_scale_bar(Canvas* canvas, float scale, bool imp) {
    static const float steps_m[]={1,2,5,10,20,50,100,200,500,1000,2000,5000};
    static const float steps_ft[]={5,10,20,50,100,200,500,1000,2000,5280,10560};
    const float* steps=imp ? steps_ft : steps_m;
    size_t n=imp ? sizeof(steps_ft)/sizeof(steps_ft[0]) : sizeof(steps_m)/sizeof(steps_m[0]);
    float unit=imp ? 0.3048f : 1.0f;
    float bar=steps[0];
    const float max_w=canvas_is_tall(canvas) ? 22.0f : 36.0f;
    for(size_t i=0;i<n;i++) {
        if(steps[i]*unit*scale<=max_w) bar=steps[i];
    }
    int w=(int)(bar*unit*scale);
    if(w<2) return;
    char b[12];
    if(imp) {
        if(bar>=5280.0f) snprintf(b,sizeof(b),"%.0fmi",(double)(bar/5280.0f));
        else snprintf(b,sizeof(b),"%.0fft",(double)bar);
    } else {
        if(bar>=1000.0f) snprintf(b,sizeof(b),"%.0fkm",(double)(bar/1000.0f));
        else snprintf(b,sizeof(b),"%.0fm",(double)bar);
    }
    int tw=(int)canvas_string_width(canvas,b);
    canvas_set_color(canvas,ColorWhite);
    const int by=body_bottom(canvas)-2; /* baseline: two clear pixels above the footer */
    canvas_draw_box(canvas,0,by-9,(size_t)(w+tw+9),11);
    canvas_set_color(canvas,ColorBlack);
    canvas_draw_line(canvas,2,by,2+w,by);
    canvas_draw_line(canvas,2,by-2,2,by);
    canvas_draw_line(canvas,2+w,by-2,2+w,by);
    canvas_draw_str(canvas,6+w,by,b);
}

static void draw_route_pass(
    Canvas* canvas, const Trail* t, double ref_lat, double ref_lon,
    const MapDraw* m, int ox, int oy) {
    UNUSED(canvas); /* the lines are drawn through m->canvas */
    float pxp=0.0f, pyp=0.0f;
    for(size_t i=0;i<t->route_count;i++) {
        float e,n,px,py;
        map_local_xy(t->route[i].lat,t->route[i].lon,ref_lat,ref_lon,&e,&n);
        map_xy(m,e,n,&px,&py);
        px+=(float)ox;
        py+=(float)oy;
        if(i>0) draw_line_vis(m,pxp,pyp,px,py,false);
        pxp=px;
        pyp=py;
    }
}

/* Saved routes of the live window, plus (in free-move) the window around the camera. */
static void map_each_segment(Trail* t, SegFn fn, void* ctx) {
    tilestore_each_segment(&t->tiles,NULL,fn,ctx);
    if(t->map_free) tilestore_each_segment(&t->view,&t->tiles,fn,ctx);
}

static const char* compass8(float e, float n) {
    static const char* const names[8]={"N","NE","E","SE","S","SW","W","NW"};
    float deg=atan2f(e,n)*57.29578f;
    if(deg<0.0f) deg+=360.0f;
    int idx=((int)((deg+22.5f)/45.0f))%8;
    return names[idx];
}

/* Stack of labels at the top right of the map; stops before the footer. */
static void push_label(Canvas* canvas, int* ly, const char* text) {
    const int limit=body_bottom(canvas)-3;
    if(*ly>limit) return;
    if(canvas_is_tall(canvas) && (int)canvas_string_width(canvas,text)>(int)canvas_width(canvas)-4) {
        const char* sp=strchr(text,' '); /* too wide for 64 px: put the first word on its own line */
        if(sp) {
            char head[24];
            size_t n=(size_t)(sp-text);
            if(n>=sizeof(head)) n=sizeof(head)-1;
            memcpy(head,text,n);
            head[n]='\0';
            draw_label_right(canvas,*ly,head);
            *ly+=9;
            if(*ly>limit) return;
            draw_label_right(canvas,*ly,sp+1);
            *ly+=9;
            return;
        }
    }
    draw_label_right(canvas,*ly,text);
    *ly+=9;
}

/* Angle (radians, clockwise from north) that should point up, or false for north up.
   Heading: where you are walking. Route: toward the next point of the way to the destination. */
static bool map_facing(Trail* t, float* h) {
    uint8_t mode=t->settings.facing;
    if(mode==FacingNorth) return false;
    if(mode==FacingRoute && t->pos_known) {
        int wi=wp_index_by_id(t,t->settings.dest_id);
        if(wi>=0) {
            float e=0.0f, n=0.0f;
            bool have=false;
            int32_t gx,gy;
            if(t->routing && t->nav_state==1 && t->nav_n>=2 && global_xy(t->lat,t->lon,&gx,&gy)) {
                float kx=cosf(rad(t->lat));
                for(uint16_t k=1;k<t->nav_n;k++) {
                    e=(float)(t->nav_path[k].x-gx)*RT_UNIT_M*kx;
                    n=(float)(t->nav_path[k].y-gy)*RT_UNIT_M;
                    if(e*e+n*n>=64.0f || k==t->nav_n-1) { /* first point 8 m or more ahead */
                        have=true;
                        break;
                    }
                }
            }
            if(!have) map_local_xy(t->wp[wi].lat,t->wp[wi].lon,t->lat,t->lon,&e,&n);
            if(e*e+n*n>=4.0f) {
                *h=atan2f(e,n);
                return true;
            }
        }
        /* no destination yet: use the walking direction */
    }
    if(t->hdg_valid) {
        *h=atan2f(t->hdg_sin,t->hdg_cos);
        return true;
    }
    return false;
}

/* Small compass shown while the map is turned: the N circles around as the map rotates. */
static void draw_compass(Canvas* canvas, float rot_c, float rot_s) {
    const int cx=10, cy=22;
    canvas_set_color(canvas,ColorWhite);
    canvas_draw_box(canvas,1,13,19,19);
    canvas_set_color(canvas,ColorBlack);
    canvas_draw_circle(canvas,cx,cy,8);
    canvas_draw_str_aligned(canvas,cx+(int)roundf(-rot_s*5.0f),cy+(int)roundf(-rot_c*5.0f),
                            AlignCenter,AlignCenter,"N");
}

/* Centred, word-wrapped text (upright screen); returns the next baseline. */
static int draw_wrapped(Canvas* canvas, int cx, int y, int max_w, int pitch, const char* text) {
    char line[32]={0};
    const char* p=text;
    while(*p) {
        const char* e=p;
        while(*e && *e!=' ') e++;
        char word[24];
        size_t wl=(size_t)(e-p);
        if(wl>=sizeof(word)) wl=sizeof(word)-1;
        memcpy(word,p,wl);
        word[wl]='\0';
        char trial[64];
        snprintf(trial,sizeof(trial),"%s%s%s",line,line[0] ? " " : "",word);
        if(line[0] && (int)canvas_string_width(canvas,trial)>max_w) {
            canvas_draw_str_aligned(canvas,cx,y,AlignCenter,AlignBottom,line);
            y+=pitch;
            strlcpy(line,word,sizeof(line));
        } else {
            strlcpy(line,trial,sizeof(line));
        }
        p=e;
        while(*p==' ') p++;
    }
    if(line[0]) {
        canvas_draw_str_aligned(canvas,cx,y,AlignCenter,AlignBottom,line);
        y+=pitch;
    }
    return y;
}

static void draw_empty_state(Canvas* canvas, const char* title, const char* hint) {
    if(canvas_is_tall(canvas)) {
        const int cx=(int)canvas_width(canvas)/2;
        canvas_set_font(canvas,FontPrimary);
        int y=draw_wrapped(canvas,cx,52,60,12,title);
        canvas_set_font(canvas,FontSecondary);
        draw_wrapped(canvas,cx,y+2,60,10,hint);
        return;
    }
    canvas_set_font(canvas,FontPrimary);
    canvas_draw_str_aligned(canvas,64,31,AlignCenter,AlignBottom,title);
    canvas_set_font(canvas,FontSecondary);
    canvas_draw_str_aligned(canvas,64,43,AlignCenter,AlignBottom,hint);
}

static void draw_live_map(Canvas* canvas, Trail* t) {
    canvas_clear(canvas);
    draw_header(canvas,t,"MAP PREVIEW");
    canvas_set_font(canvas,FontSecondary);

    bool live=t->fix;
    bool have_you=t->pos_known; /* live fix, or the last known position */
    if(!have_you && t->route_count==0) {
        draw_empty_state(canvas,"Waiting for GPS","Map needs a fix or a route");
        return;
    }

    const TrailSettings* st=&t->settings;
    bool free_mode=t->map_free;
    bool follow=(st->map_view==MapViewFollow);
    bool show_trip=(st->map_layer!=MapLayerSaved);
    bool show_saved=(st->map_layer!=MapLayerRoute);
    bool imp=st->imperial;
    int wi_dest=wp_index_by_id(t,st->dest_id);

    /* Reference point: the camera in free-move, else you, else the last route point. */
    double ref_lat,ref_lon;
    if(free_mode) {
        ref_lat=t->cam_lat;
        ref_lon=t->cam_lon;
    } else if(have_you) {
        ref_lat=t->lat;
        ref_lon=t->lon;
    } else {
        ref_lat=t->route[t->route_count-1].lat;
        ref_lon=t->route[t->route_count-1].lon;
    }

    MapDraw m;
    memset(&m,0,sizeof(m));
    m.canvas=canvas;
    if(!global_xy(ref_lat,ref_lon,&m.ref_gx,&m.ref_gy)) return;
    m.unit_h_m=111320.0f/(CELLS_PER_DEG*(float)RT_UNITS);
    m.unit_w_m=m.unit_h_m*cosf(rad(ref_lat));
    m.rot_c=1.0f;
    m.rot_s=0.0f;

    /* turn the map only in the follow view (the overview and free-move stay north up) */
    float fh=0.0f;
    bool rotate=(follow && !free_mode && map_facing(t,&fh));
    if(rotate) {
        m.rot_c=cosf(fh);
        m.rot_s=sinf(fh);
    }

    if(follow || free_mode) {
        m.scale=124.0f/zoom_span_m[st->zoom];
        m.center_e=0.0f;
        m.center_n=rotate ? (canvas_is_tall(canvas) ? 20.0f : 8.0f)/m.scale : 0.0f; /* you sit a little below the middle: more view ahead */
    } else {
        if(show_trip) {
            for(size_t i=0;i<t->route_count;i++) {
                float e,n;
                map_local_xy(t->route[i].lat,t->route[i].lon,ref_lat,ref_lon,&e,&n);
                ext_add(&m.ext,e,n);
            }
        }
        if(show_saved) map_each_segment(t,seg_extent_cb,&m);
        if(have_you) ext_add(&m.ext,0.0f,0.0f);
        if(wi_dest>=0) { /* keep the destination in view */
            float e,n;
            map_local_xy(t->wp[wi_dest].lat,t->wp[wi_dest].lon,ref_lat,ref_lon,&e,&n);
            ext_add(&m.ext,e,n);
        }
        if(!m.ext.have) ext_add(&m.ext,0.0f,0.0f);

        m.center_e=(m.ext.min_e+m.ext.max_e)/2.0f;
        m.center_n=(m.ext.min_n+m.ext.max_n)/2.0f;
        float span_e=m.ext.max_e-m.ext.min_e;
        float span_n=m.ext.max_n-m.ext.min_n;
        if(span_e<20.0f) span_e=20.0f;
        if(span_n<20.0f) span_n=20.0f;
        span_e*=1.15f;
        span_n*=1.15f;
        float sx=(float)((int)canvas_width(canvas)-4)/span_e;
        float sy=(float)(body_bottom(canvas)-UI_BODY_TOP-4)/span_n;
        m.scale=sx<sy ? sx : sy;
    }

    map_areas_setup(t,&m,ref_lat,ref_lon);

    /* saved routes: dotted */
    if(show_saved) map_each_segment(t,seg_draw_cb,&m);

    /* this trip: solid, with a white lane around it when saved routes are underneath */
    if(show_trip) {
        if(show_saved) {
            canvas_set_color(canvas,ColorWhite);
            draw_route_pass(canvas,t,ref_lat,ref_lon,&m,1,0);
            draw_route_pass(canvas,t,ref_lat,ref_lon,&m,-1,0);
            draw_route_pass(canvas,t,ref_lat,ref_lon,&m,0,1);
            draw_route_pass(canvas,t,ref_lat,ref_lon,&m,0,-1);
            canvas_set_color(canvas,ColorBlack);
        }
        draw_route_pass(canvas,t,ref_lat,ref_lon,&m,0,0);

        if(t->route_count>0) {
            float e,n,px,py;
            map_local_xy(t->route[0].lat,t->route[0].lon,ref_lat,ref_lon,&e,&n);
            map_xy(&m,e,n,&px,&py);
            if(!map_px_in_area(&m,px,py)) draw_marker(canvas,px,py,MarkerStart);
        }
    }

    /* way to the destination: a heavier line along saved routes (dotted straight line if none) */
    if(wi_dest>=0 && t->routing) {
        if(t->nav_state==1 && t->nav_n>=2) {
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
        } else if(have_you) {
            float e,n,e2,n2,ax,ay,bx,by;
            map_local_xy(t->lat,t->lon,ref_lat,ref_lon,&e,&n);
            map_local_xy(t->wp[wi_dest].lat,t->wp[wi_dest].lon,ref_lat,ref_lon,&e2,&n2);
            map_xy(&m,e,n,&ax,&ay);
            map_xy(&m,e2,n2,&bx,&by);
            draw_dotted_line(canvas,ax,ay,bx,by);
        }
    }

    for(uint8_t k=0;k<m.ac_n;k++) draw_area_circle(canvas,m.ac_x[k],m.ac_y[k],m.ac_r[k]);

    /* saved locations */
    for(int i=0;i<(int)t->wp_count;i++) {
        float e,n,px,py;
        map_local_xy(t->wp[i].lat,t->wp[i].lon,ref_lat,ref_lon,&e,&n);
        map_xy(&m,e,n,&px,&py);
        draw_marker(canvas,px,py,(i==wi_dest) ? MarkerDest : MarkerPin);
    }

    if(have_you) {
        float e,n,px,py;
        map_local_xy(t->lat,t->lon,ref_lat,ref_lon,&e,&n);
        map_xy(&m,e,n,&px,&py);
        draw_marker(canvas,px,py,live ? MarkerYou : MarkerLast);
    }

    draw_scale_bar(canvas,m.scale,imp);
    if(rotate) draw_compass(canvas,m.rot_c,m.rot_s);

    int ly=(rotate && canvas_is_tall(canvas)) ? 40 : 21; /* upright: start below the compass */
    char b[32], d[16];
    if(t->toast[0] && (int32_t)(furi_get_tick()-t->toast_until)<0) push_label(canvas,&ly,t->toast);
    bool dest_says_in=(!free_mode && wi_dest>=0 && t->wp[wi_dest].radius_m>0 && t->wp[wi_dest].id==t->area_in_id);
    if(t->area_in_id && !dest_says_in) {
        int ai=wp_index_by_id(t,t->area_in_id);
        if(ai>=0) {
            snprintf(b,sizeof(b),"IN %s",t->wp[ai].name);
            push_label(canvas,&ly,b);
        }
    }

    if(free_mode) {
        /* crosshair marks the point being looked at */
        canvas_set_color(canvas,ColorWhite);
        canvas_draw_box(canvas,MAP_CX-4,MAP_CY-1,9,3);
        canvas_draw_box(canvas,MAP_CX-1,MAP_CY-4,3,9);
        canvas_set_color(canvas,ColorBlack);
        canvas_draw_line(canvas,MAP_CX-3,MAP_CY,MAP_CX+3,MAP_CY);
        canvas_draw_line(canvas,MAP_CX,MAP_CY-3,MAP_CX,MAP_CY+3);

        int near=wp_near(t,ref_lat,ref_lon,25.0f);
        if(near>=0) {
            snprintf(b,sizeof(b),"%s hold OK:%s",t->wp[near].name,
                     (t->wp[near].id==st->dest_id) ? "stop" : "go");
            push_label(canvas,&ly,b);
        }
        if(have_you) {
            float e,n;
            map_local_xy(ref_lat,ref_lon,t->lat,t->lon,&e,&n); /* crosshair relative to you */
            float dm=sqrtf(e*e+n*n);
            if(dm<2.0f) {
                snprintf(b,sizeof(b),"at you");
            } else {
                fmt_dist(d,sizeof(d),dm,imp);
                snprintf(b,sizeof(b),"%s %s",d,compass8(e,n));
            }
            push_label(canvas,&ly,b);
        }
        int16_t alt=ALT_UNKNOWN;
        if(rt_nearest_alt(t,ref_lat,ref_lon,20.0f,&alt)) {
            if(alt!=ALT_UNKNOWN)
                snprintf(b,sizeof(b),"Alt %.0f%s",(double)alt_conv((float)alt/ALT_SCALE,imp),alt_unit(imp));
            else
                snprintf(b,sizeof(b),"Alt --");
        } else {
            snprintf(b,sizeof(b),"no route");
        }
        push_label(canvas,&ly,b);
    } else if(wi_dest>=0) {
        const Waypoint* w=&t->wp[wi_dest];
        float dm=t->nav_direct_m;
        if(have_you) {
            float e,n;
            map_local_xy(w->lat,w->lon,t->lat,t->lon,&e,&n);
            dm=sqrtf(e*e+n*n);
            if(w->radius_m>0) { /* an area: distance to its edge */
                dm-=(float)w->radius_m;
                if(dm<0.0f) dm=0.0f;
            }
        }
        if(have_you && dm<=0.0f && w->radius_m>0) {
            snprintf(b,sizeof(b),"in %s",w->name);
        } else if(have_you && dm<20.0f) {
            snprintf(b,sizeof(b),"%s here",w->name);
        } else if(!t->routing) {
            fmt_dist(d,sizeof(d),dm,imp);
            snprintf(b,sizeof(b),"%s %s",w->name,d);
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
    }
    if(!live) push_label(canvas,&ly,"NO FIX");
}
