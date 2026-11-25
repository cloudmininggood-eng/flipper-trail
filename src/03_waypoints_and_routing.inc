/* flipper_trail module: waypoints and routing. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---- Saved locations (waypoints) ---- */
static void toast_set(Trail* t, const char* text) {
    strlcpy(t->toast,text,sizeof(t->toast));
    t->toast_until=furi_get_tick()+2500;
}

static int wp_index_by_id(const Trail* t, uint8_t id) {
    if(id==0) return -1;
    for(int i=0;i<(int)t->wp_count;i++) {
        if(t->wp[i].id==id) return i;
    }
    return -1;
}

static void wp_save(const Trail* t) {
    WpHeader h={.magic={'F','W','P'},.version=3,.count=t->wp_count,.next_id=t->wp_next_id,.reserved=0};
    WpRec3 rec[WP_MAX];
    for(uint8_t i=0;i<t->wp_count && i<WP_MAX;i++) {
        memset(&rec[i],0,sizeof(WpRec3));
        rec[i].lat_e7=deg_to_e7(t->wp[i].lat);
        rec[i].lon_e7=deg_to_e7(t->wp[i].lon);
        rec[i].id=t->wp[i].id;
        memcpy(rec[i].name,t->wp[i].name,sizeof(rec[i].name));
        rec[i].radius_m=t->wp[i].radius_m;
    }
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,WP_PATH,FSAM_WRITE,FSOM_CREATE_ALWAYS)) {
        storage_file_write(f,&h,sizeof(h));
        if(t->wp_count) storage_file_write(f,rec,(uint16_t)(sizeof(WpRec3)*t->wp_count));
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

static void wp_load(Trail* t) {
    t->wp_count=0;
    t->wp_next_id=1;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,WP_PATH,FSAM_READ,FSOM_OPEN_EXISTING)) {
        WpHeader h;
        if(storage_file_read(f,&h,sizeof(h))==sizeof(h) && memcmp(h.magic,"FWP",3)==0 &&
           h.version>=1 && h.version<=3 && h.count<=WP_MAX) {
            /* versions 1 and 2 hold 16-byte records (floats, then 1e-7 degree); version 3 adds a radius */
            size_t rsz=(h.version==3) ? sizeof(WpRec3) : sizeof(WpRec);
            uint8_t raw[WP_MAX*sizeof(WpRec3)];
            size_t body=rsz*h.count;
            if(body==0 || storage_file_read(f,raw,(uint16_t)body)==body) {
                for(uint8_t i=0;i<h.count;i++) {
                    Waypoint* w=&t->wp[i];
                    memset(w,0,sizeof(Waypoint));
                    if(h.version==1) {
                        WpRecV1 old;
                        memcpy(&old,raw+i*rsz,sizeof(old));
                        w->lat=(double)old.lat;
                        w->lon=(double)old.lon;
                        w->id=old.id;
                        memcpy(w->name,old.name,sizeof(w->name));
                    } else if(h.version==2) {
                        WpRec r2;
                        memcpy(&r2,raw+i*rsz,sizeof(r2));
                        w->lat=e7_to_deg(r2.lat_e7);
                        w->lon=e7_to_deg(r2.lon_e7);
                        w->id=r2.id;
                        memcpy(w->name,r2.name,sizeof(w->name));
                    } else {
                        WpRec3 r3;
                        memcpy(&r3,raw+i*rsz,sizeof(r3));
                        w->lat=e7_to_deg(r3.lat_e7);
                        w->lon=e7_to_deg(r3.lon_e7);
                        w->id=r3.id;
                        memcpy(w->name,r3.name,sizeof(w->name));
                        w->radius_m=(r3.radius_m<=area_r[AREA_R_COUNT-1]) ? r3.radius_m : 0;
                    }
                    w->name[sizeof(w->name)-1]='\0';
                }
                t->wp_count=h.count;
                t->wp_next_id=h.next_id ? h.next_id : 1;
            }
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

/* New saved location; returns its index, or -1 if the list is full. */
static int wp_add(Trail* t, double lat, double lon) {
    if(t->wp_count>=WP_MAX) return -1;
    uint8_t id=t->wp_next_id ? t->wp_next_id : 1;
    while(wp_index_by_id(t,id)>=0) {
        id++;
        if(id==0) id=1;
    }
    Waypoint* w=&t->wp[t->wp_count];
    memset(w,0,sizeof(Waypoint));
    w->lat=lat;
    w->lon=lon;
    w->id=id;
    snprintf(w->name,sizeof(w->name),"WP%u",(unsigned)id);
    t->wp_next_id=(uint8_t)(id==255 ? 1 : id+1);
    t->wp_count++;
    wp_save(t);
    return (int)t->wp_count-1;
}

/* Removes a location. If it was the destination, the destination is cleared
   (the caller saves the settings). */
static void wp_delete(Trail* t, int idx) {
    if(idx<0 || idx>=(int)t->wp_count) return;
    if(t->settings.dest_id==t->wp[idx].id) {
        t->settings.dest_id=0;
        t->routing=false;
        t->nav_dirty=true;
    }
    for(int i=idx;i<(int)t->wp_count-1;i++) t->wp[i]=t->wp[i+1];
    t->wp_count--;
    wp_save(t);
}

/* Index of the saved location within max_m of a place, or -1. */
static int wp_near(const Trail* t, double lat, double lon, float max_m) {
    int best=-1;
    float bd=max_m;
    float kx=cosf(rad((float)lat));
    for(int i=0;i<(int)t->wp_count;i++) {
        float e=(float)((t->wp[i].lon-lon)*(double)111320.0)*kx;
        float n=(float)((t->wp[i].lat-lat)*(double)111320.0);
        float d=sqrtf(e*e+n*n);
        if(d<=bd) {
            bd=d;
            best=i;
        }
    }
    return best;
}

/* ---- Routing: shortest way along saved routes from here to the destination ----
   Every stored route point is a node. Neighbouring points of a stroke are joined, and any
   two points closer than the "link gap" are joined too (that is how separate recordings
   connect where they cross or touch). You and the destination join the network if a route
   point is within a few link gaps. Dijkstra then finds the shortest chain. */
#define NAV_INF 0xFFFFFFFFu

typedef struct {
    int32_t *gx, *gy;           /* node positions, global units (+2 virtual: start, destination) */
    uint16_t *ea, *eb, *el;     /* stroke edges: ends and length in decimetres */
    uint16_t *parent;
    uint32_t* dist;             /* decimetres from the start */
    uint8_t* done;
    uint16_t n, ne;
    bool full;                  /* node limit reached while reading tiles */
} NavGraph;

static void nav_graph_free(NavGraph* g) {
    free(g->gx); free(g->gy);
    free(g->ea); free(g->eb); free(g->el);
    free(g->parent); free(g->dist); free(g->done);
    memset(g,0,sizeof(NavGraph));
}

static bool nav_graph_alloc(NavGraph* g) {
    memset(g,0,sizeof(NavGraph));
    g->gx=malloc(sizeof(int32_t)*(NAV_NODES+2));
    g->gy=malloc(sizeof(int32_t)*(NAV_NODES+2));
    g->ea=malloc(sizeof(uint16_t)*NAV_EDGES);
    g->eb=malloc(sizeof(uint16_t)*NAV_EDGES);
    g->el=malloc(sizeof(uint16_t)*NAV_EDGES);
    g->parent=malloc(sizeof(uint16_t)*(NAV_NODES+2));
    g->dist=malloc(sizeof(uint32_t)*(NAV_NODES+2));
    g->done=malloc(NAV_NODES+2);
    if(!g->gx || !g->gy || !g->ea || !g->eb || !g->el || !g->parent || !g->dist || !g->done) {
        nav_graph_free(g);
        return false;
    }
    return true;
}

static uint16_t nav_len_dm(int32_t dx, int32_t dy, float kx) {
    float fx=(float)dx*kx, fy=(float)dy;
    float d=sqrtf(fx*fx+fy*fy)*RT_UNIT_M*10.0f;
    return d>65535.0f ? (uint16_t)65535 : (uint16_t)d;
}

/* Index of the node at this position (added if new), or -1 when the node limit is hit. */
static int nav_node(NavGraph* g, int32_t x, int32_t y) {
    for(uint16_t i=0;i<g->n;i++) {
        if(g->gx[i]==x && g->gy[i]==y) return (int)i;
    }
    if(g->n>=NAV_NODES) {
        g->full=true;
        return -1;
    }
    g->gx[g->n]=x;
    g->gy[g->n]=y;
    return (int)g->n++;
}

static void nav_edge(NavGraph* g, int a, int b, float kx) {
    if(a<0 || b<0 || a==b || g->ne>=NAV_EDGES) return;
    g->ea[g->ne]=(uint16_t)a;
    g->eb[g->ne]=(uint16_t)b;
    g->el[g->ne]=nav_len_dm(g->gx[a]-g->gx[b],g->gy[a]-g->gy[b],kx);
    g->ne++;
}

/* Dijkstra from the start node (index nreal) to the destination node (nreal+1). */
static bool nav_solve(NavGraph* g, uint16_t nreal, float kx, float link, float snap, float area_r_m) {
    uint16_t V=(uint16_t)(nreal+2);
    uint16_t start=nreal;
    uint16_t dst=(uint16_t)(nreal+1);
    for(uint16_t i=0;i<V;i++) {
        g->dist[i]=NAV_INF;
        g->parent[i]=0xFFFF;
        g->done[i]=0;
    }
    g->dist[start]=0;
    int32_t limy_l=(int32_t)(link/RT_UNIT_M)+1, limx_l=(int32_t)(link/(RT_UNIT_M*kx))+1;
    int32_t limy_s=(int32_t)(snap/RT_UNIT_M)+1, limx_s=(int32_t)(snap/(RT_UNIT_M*kx))+1;
    uint32_t link_dm=(uint32_t)(link*10.0f), snap_dm=(uint32_t)(snap*10.0f);
    uint32_t area_dm=(uint32_t)(area_r_m*10.0f); /* 0 = the destination is a point */
    for(;;) {
        uint32_t best=NAV_INF;
        int u=-1;
        for(uint16_t i=0;i<V;i++) {
            if(!g->done[i] && g->dist[i]<best) {
                best=g->dist[i];
                u=(int)i;
            }
        }
        if(u<0) break;
        g->done[u]=1;
        if(u==(int)dst) break;
        for(uint16_t e=0;e<g->ne;e++) {
            int v=-1;
            if(g->ea[e]==u) v=g->eb[e];
            else if(g->eb[e]==u) v=g->ea[e];
            if(v<0 || g->done[v]) continue;
            uint32_t nd=best+g->el[e];
            if(nd<g->dist[v]) {
                g->dist[v]=nd;
                g->parent[v]=(uint16_t)u;
            }
        }
        for(uint16_t v=0;v<V;v++) {
            if(g->done[v]) continue;
            bool virt=(u>=(int)nreal) || (v>=nreal);
            int32_t dx=g->gx[v]-g->gx[u];
            int32_t dy=g->gy[v]-g->gy[u];
            if(dx<0) dx=-dx;
            if(dy<0) dy=-dy;
            if(area_dm>0 && v==dst) {
                /* an area is reached at its edge: cost is the distance to the circle, 0 inside */
                uint16_t cdm=nav_len_dm(dx,dy,kx);
                uint32_t edge=((uint32_t)cdm>area_dm) ? (uint32_t)cdm-area_dm : 0u;
                if(edge>snap_dm) continue;
                uint32_t nde=best+edge;
                if(nde<g->dist[v]) {
                    g->dist[v]=nde;
                    g->parent[v]=(uint16_t)u;
                }
                continue;
            }
            if(dx>(virt ? limx_s : limx_l) || dy>(virt ? limy_s : limy_l)) continue;
            uint16_t dm=nav_len_dm(dx,dy,kx);
            if((uint32_t)dm>(virt ? snap_dm : link_dm)) continue;
            uint32_t nd=best+dm;
            if(nd<g->dist[v]) {
                g->dist[v]=nd;
                g->parent[v]=(uint16_t)u;
            }
        }
    }
    return g->dist[dst]!=NAV_INF;
}

static void nav_set_state(Trail* t, uint8_t state) {
    furi_mutex_acquire(t->mutex,FuriWaitForever);
    t->nav_state=state;
    t->nav_n=0;
    furi_mutex_release(t->mutex);
}

/* Build the network from the tile files between you and the destination, then solve it. */
static void nav_compute(Trail* t) {
    t->nav_dirty=false;
    t->nav_tick=furi_get_tick();
    int wi=wp_index_by_id(t,t->settings.dest_id);
    int32_t sx,sy,ex,ey;
    if(wi<0 || !t->pos_known || !global_xy(t->lat,t->lon,&sx,&sy) ||
       !global_xy(t->wp[wi].lat,t->wp[wi].lon,&ex,&ey)) {
        nav_set_state(t,0);
        return;
    }
    t->nav_from_gx=sx;
    t->nav_from_gy=sy;
    float kx=cosf(rad(t->lat));
    {
        float dx=(float)(ex-sx)*kx, dy=(float)(ey-sy);
        t->nav_direct_m=sqrtf(dx*dx+dy*dy)*RT_UNIT_M;
        if(t->wp[wi].radius_m>0) { /* an area: straight distance to its edge */
            t->nav_direct_m-=(float)t->wp[wi].radius_m;
            if(t->nav_direct_m<0.0f) t->nav_direct_m=0.0f;
        }
    }

    int32_t tx0=((sx<ex ? sx : ex)>>RT_TILE_SHIFT)-1;
    int32_t tx1=((sx<ex ? ex : sx)>>RT_TILE_SHIFT)+1;
    int32_t ty0=((sy<ey ? sy : ey)>>RT_TILE_SHIFT)-1;
    int32_t ty1=((sy<ey ? ey : sy)>>RT_TILE_SHIFT)+1;
    if(tx0<0) tx0=0;
    if(ty0<0) ty0=0;
    if((tx1-tx0+1)*(ty1-ty0+1)>400) {
        nav_set_state(t,3); /* too far */
        return;
    }

    tilestore_flush_all(&t->tiles); /* the files must hold what has been recorded so far */
    NavGraph g;
    RTile* tile=malloc(sizeof(RTile));
    if(!tile || !nav_graph_alloc(&g)) {
        free(tile);
        nav_set_state(t,3);
        return;
    }

    Storage* storage=furi_record_open(RECORD_STORAGE);
    int32_t last_shx=-1, last_shy=-1;
    bool shard_ok=false;
    for(int32_t ty=ty0;ty<=ty1;ty++) {
        for(int32_t tx=tx0;tx<=tx1;tx++) {
            if((tx>>5)!=last_shx || (ty>>5)!=last_shy) {
                char dir[48];
                FileInfo fi;
                rt_path(dir,sizeof(dir),(uint32_t)tx,(uint32_t)ty,true);
                shard_ok=(storage_common_stat(storage,dir,&fi)==FSE_OK);
                last_shx=tx>>5;
                last_shy=ty>>5;
            }
            if(!shard_ok) continue;
            char path[64];
            rt_path(path,sizeof(path),(uint32_t)tx,(uint32_t)ty,false);
            if(!rtile_read(tile,path)) continue;
            int prev=-1;
            for(int i=0;i<(int)tile->n;i++) {
                if(rt_is_break(&tile->p[i])) { prev=-1; continue; }
                int32_t gx=(tx<<RT_TILE_SHIFT)+tile->p[i].x;
                int32_t gy=(ty<<RT_TILE_SHIFT)+tile->p[i].y;
                int idx=nav_node(&g,gx,gy);
                if(idx<0) { prev=-1; continue; }
                if(prev>=0 && idx!=prev) nav_edge(&g,prev,idx,kx);
                prev=idx;
            }
        }
    }
    furi_record_close(RECORD_STORAGE);
    free(tile);

    uint16_t nreal=g.n;
    g.gx[nreal]=sx;
    g.gy[nreal]=sy;
    g.gx[nreal+1]=ex;
    g.gy[nreal+1]=ey;
    float link=link_m[t->settings.link_idx];
    float snap=link*3.0f;
    if(snap<40.0f) snap=40.0f;

    if(!nav_solve(&g,nreal,kx,link,snap,(float)t->wp[wi].radius_m)) {
        nav_set_state(t,g.full ? 3 : 2);
        nav_graph_free(&g);
        return;
    }

    if(t->wp[wi].radius_m>0) {
        /* the way ends where it meets the circle, not at its centre */
        uint16_t pv=g.parent[nreal+1];
        if(pv!=0xFFFF) {
            float dxu=(float)(g.gx[pv]-ex)*kx, dyu=(float)(g.gy[pv]-ey);
            float dmeters=sqrtf(dxu*dxu+dyu*dyu)*RT_UNIT_M;
            float rr=(float)t->wp[wi].radius_m;
            if(dmeters>rr) {
                float f=rr/dmeters;
                g.gx[nreal+1]=ex+(int32_t)((float)(g.gx[pv]-ex)*f);
                g.gy[nreal+1]=ey+(int32_t)((float)(g.gy[pv]-ey)*f);
            } else {
                g.gx[nreal+1]=g.gx[pv];
                g.gy[nreal+1]=g.gy[pv];
            }
        }
    }

    /* walk back from the destination to the start */
    uint16_t V=(uint16_t)(nreal+2);
    uint16_t* chain=malloc(sizeof(uint16_t)*V);
    if(!chain) {
        nav_set_state(t,3);
        nav_graph_free(&g);
        return;
    }
    int count=0;
    uint16_t v=(uint16_t)(nreal+1);
    while(v!=0xFFFF && count<(int)V) {
        chain[count++]=v;
        if(v==nreal) break;
        v=g.parent[v];
    }
    int out_n=count<NAV_PATH_MAX ? count : NAV_PATH_MAX;
    furi_mutex_acquire(t->mutex,FuriWaitForever);
    for(int k=0;k<out_n;k++) {
        int idx=(out_n==count) ? (count-1-k) : ((count-1)-(k*(count-1))/(out_n-1));
        t->nav_path[k].x=g.gx[chain[idx]];
        t->nav_path[k].y=g.gy[chain[idx]];
    }
    t->nav_n=(uint16_t)out_n;
    t->nav_route_m=(float)g.dist[nreal+1]/10.0f;
    t->nav_state=1;
    furi_mutex_release(t->mutex);
    free(chain);
    nav_graph_free(&g);
}
