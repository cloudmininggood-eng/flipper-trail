/* flipper_trail module: geometry and tiles. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---- geometry on tile-local points ---- */
static bool rt_is_break(const RPt* p) { return p->x==RT_BREAK; }

static float pt_seg_dist2(float px, float py, float ax, float ay, float bx, float by, float kx) {
    float dx=(bx-ax)*kx, dy=by-ay;
    float qx=(px-ax)*kx, qy=py-ay;
    float len2=dx*dx+dy*dy;
    float t=0.0f;
    if(len2>1e-6f) {
        t=(qx*dx+qy*dy)/len2;
        if(t<0.0f) t=0.0f;
        if(t>1.0f) t=1.0f;
    }
    float ex=qx-t*dx, ey=qy-t*dy;
    return ex*ex+ey*ey;
}

/* Is (px,py) within sqrt(thr2) units of a stroke among p[0..n)? */
static bool rpts_near(const RPt* p, int n, float px, float py, float thr2, float kx) {
    for(int i=1;i<n;i++) {
        if(rt_is_break(&p[i]) || rt_is_break(&p[i-1])) continue;
        if(pt_seg_dist2(px,py,(float)p[i-1].x,(float)p[i-1].y,(float)p[i].x,(float)p[i].y,kx)<=thr2)
            return true;
    }
    return false;
}

/* Merge cnt altitude samples averaging `alt` into a stored point: a running mean weighted
   by how many samples each side stands for. */
static void rpt_merge_alt(RPt* dst, int16_t alt, uint8_t cnt) {
    if(alt==ALT_UNKNOWN || cnt==0) return;
    if(dst->alt==ALT_UNKNOWN || dst->n==0) {
        dst->alt=alt;
        dst->n=cnt;
        return;
    }
    int32_t total=(int32_t)dst->n+(int32_t)cnt;
    int32_t sum=(int32_t)dst->alt*(int32_t)dst->n+(int32_t)alt*(int32_t)cnt;
    int32_t mean=(sum>=0) ? (sum+total/2)/total : -((-sum+total/2)/total);
    dst->alt=(int16_t)mean;
    dst->n=(uint8_t)(total>255 ? 255 : total);
}

/* Fold an altitude into the stored path near (px,py): the nearest stretch within thr units,
   merged into whichever end of it is closer. Returns true if something changed. */
static bool rpts_blend(RPt* p, int n, float px, float py, int16_t alt, uint8_t cnt,
                       float thr2, float kx) {
    if(alt==ALT_UNKNOWN || cnt==0) return false;
    int best=-1;
    float bd=thr2;
    for(int i=1;i<n;i++) {
        if(rt_is_break(&p[i]) || rt_is_break(&p[i-1])) continue;
        float ax=(float)p[i-1].x, ay=(float)p[i-1].y;
        float dx=((float)p[i].x-ax)*kx, dy=(float)p[i].y-ay;
        float qx=(px-ax)*kx, qy=py-ay;
        float len2=dx*dx+dy*dy;
        float t=0.0f;
        if(len2>1e-6f) {
            t=(qx*dx+qy*dy)/len2;
            if(t<0.0f) t=0.0f;
            if(t>1.0f) t=1.0f;
        }
        float ex=qx-t*dx, ey=qy-t*dy;
        float d=ex*ex+ey*ey;
        if(d<=bd) {
            bd=d;
            best=(t<0.5f) ? i-1 : i;
        }
    }
    if(best<0) return false;
    int16_t before=p[best].alt;
    uint8_t before_n=p[best].n;
    rpt_merge_alt(&p[best],alt,cnt);
    return p[best].alt!=before || p[best].n!=before_n;
}

typedef struct {
    uint8_t keep[RT_MAX];
    int16_t stk[64][2];
} DpScratch; /* kept off the small app stack */

/* Douglas-Peucker: mark the points of one stroke that must stay. */
static void dp_simplify(const RPt* p, int n, float eps, float kx, DpScratch* sc) {
    memset(sc->keep,0,(size_t)n);
    sc->keep[0]=1;
    sc->keep[n-1]=1;
    float eps2=eps*eps;
    int sp=0;
    sc->stk[sp][0]=0;
    sc->stk[sp][1]=(int16_t)(n-1);
    sp++;
    while(sp>0) {
        sp--;
        int i0=sc->stk[sp][0], i1=sc->stk[sp][1];
        if(i1-i0<2) continue;
        float maxd=0.0f;
        int idx=-1;
        for(int i=i0+1;i<i1;i++) {
            float d=pt_seg_dist2((float)p[i].x,(float)p[i].y,(float)p[i0].x,(float)p[i0].y,
                                 (float)p[i1].x,(float)p[i1].y,kx);
            if(d>maxd) { maxd=d; idx=i; }
        }
        if(idx>=0 && maxd>eps2) {
            sc->keep[idx]=1;
            if(sp<62) {
                sc->stk[sp][0]=(int16_t)i0; sc->stk[sp][1]=(int16_t)idx; sp++;
                sc->stk[sp][0]=(int16_t)idx; sc->stk[sp][1]=(int16_t)i1; sp++;
            } else {
                for(int i=i0;i<=i1;i++) sc->keep[i]=1; /* stack full: keep the range as is */
            }
        }
    }
}

/* Tidy a tile in place: straighten strokes and drop strokes that repeat earlier ones
   (every kept point within thr units of what is already kept). thr=0: no merging. */
static void rtile_compact(RTile* tl, float eps, float thr, float kx) {
    DpScratch* sc=malloc(sizeof(DpScratch));
    if(!sc) return;
    int n=tl->n;
    int w=0, r=0;
    bool alt_changed=false;
    float thr2=thr*thr;
    const RPt brk={RT_BREAK,0,0,0,0};
    while(r<n) {
        int e=r;
        while(e<n && !rt_is_break(&tl->p[e])) e++;
        int len=e-r;
        if(len>=2) {
            dp_simplify(&tl->p[r],len,eps,kx,sc);
            bool covered=(thr>0.0f && w>0);
            int prev=-1;
            for(int i=0;i<len && covered;i++) {
                if(!sc->keep[i]) continue;
                const RPt* pi=&tl->p[r+i];
                if(!rpts_near(tl->p,w,(float)pi->x,(float)pi->y,thr2,kx)) {
                    covered=false;
                } else if(prev>=0) {
                    const RPt* pp=&tl->p[r+prev];
                    float mx=0.5f*((float)pi->x+(float)pp->x);
                    float my=0.5f*((float)pi->y+(float)pp->y);
                    if(!rpts_near(tl->p,w,mx,my,thr2,kx)) covered=false;
                }
                prev=i;
            }
            if(!covered) {
                /* points the straightening drops: average their altitude into the nearest kept one */
                for(int i=0;i<len;i++) {
                    if(sc->keep[i]) continue;
                    RPt* pi=&tl->p[r+i];
                    if(pi->alt==ALT_UNKNOWN || pi->n==0) continue;
                    int pk=-1, nk=-1;
                    for(int k=i-1;k>=0;k--) {
                        if(sc->keep[k]) { pk=k; break; }
                    }
                    for(int k=i+1;k<len;k++) {
                        if(sc->keep[k]) { nk=k; break; }
                    }
                    int tgt=pk;
                    if(pk<0) {
                        tgt=nk;
                    } else if(nk>=0) {
                        float dpx=((float)pi->x-(float)tl->p[r+pk].x)*kx, dpy=(float)pi->y-(float)tl->p[r+pk].y;
                        float dnx=((float)pi->x-(float)tl->p[r+nk].x)*kx, dny=(float)pi->y-(float)tl->p[r+nk].y;
                        if(dnx*dnx+dny*dny<dpx*dpx+dpy*dpy) tgt=nk;
                    }
                    if(tgt>=0) rpt_merge_alt(&tl->p[r+tgt],pi->alt,pi->n);
                }
                for(int i=0;i<len;i++) {
                    if(sc->keep[i]) tl->p[w++]=tl->p[r+i];
                }
                tl->p[w++]=brk;
            } else if(thr>0.0f) {
                /* repeated path: average every point's altitude into the stored path, then drop it */
                for(int i=0;i<len;i++) {
                    const RPt* pi=&tl->p[r+i];
                    if(rpts_blend(tl->p,w,(float)pi->x,(float)pi->y,pi->alt,pi->n,thr2,kx))
                        alt_changed=true;
                }
            }
        }
        r=e+1;
    }
    if(w>0 && rt_is_break(&tl->p[w-1])) w--;
    if(w!=n || alt_changed) {
        tl->n=(uint16_t)w;
        tl->dirty=true;
    }
    free(sc);
}

/* Add one segment (tile-local) to a tile. Returns the number of points stored (0 if the
   path is already known). thr_units = 0 disables the repeated-path check. */
static int rtile_add_segment(RTile* tl, RPt a, RPt b, float thr_units, float kx) {
    const RPt brk={RT_BREAK,0,0,0,0};
    /* A segment that continues the stroke being drawn is always kept: otherwise a path
       would be judged "already known" because of its own tail and come out in pieces. */
    bool continuing=(tl->n>0 && !rt_is_break(&tl->p[tl->n-1]) &&
                     tl->p[tl->n-1].x==a.x && tl->p[tl->n-1].y==a.y);
    if(!continuing && thr_units>0.0f && tl->n>=2) {
        float thr2=thr_units*thr_units;
        float mx=0.5f*((float)a.x+(float)b.x);
        float my=0.5f*((float)a.y+(float)b.y);
        if(rpts_near(tl->p,tl->n,(float)a.x,(float)a.y,thr2,kx) &&
           rpts_near(tl->p,tl->n,(float)b.x,(float)b.y,thr2,kx) &&
           rpts_near(tl->p,tl->n,mx,my,thr2,kx)) {
            /* known path: fold this pass's altitude into the stored points */
            if(rpts_blend(tl->p,tl->n,(float)b.x,(float)b.y,b.alt,b.n,thr2,kx)) tl->dirty=true;
            return 0;
        }
    }
    for(int attempt=0;attempt<2;attempt++) {
        bool open_stroke=(tl->n>0 && !rt_is_break(&tl->p[tl->n-1]));
        bool extend=open_stroke && tl->p[tl->n-1].x==a.x && tl->p[tl->n-1].y==a.y;
        int need=extend ? 1 : (open_stroke ? 3 : 2);
        if(tl->n+need<=RT_MAX) {
            if(!extend) {
                if(open_stroke) tl->p[tl->n++]=brk;
                tl->p[tl->n++]=a;
            }
            tl->p[tl->n++]=b;
            tl->dirty=true;
            return extend ? 1 : 2;
        }
        if(attempt==0) {
            float eps=RT_OPT_EPS_M/RT_UNIT_M;
            float thr=thr_units>0.0f ? thr_units : RT_OPT_THR_M/RT_UNIT_M;
            rtile_compact(tl,eps,thr,kx); /* full: make room by tidying */
        }
    }
    return 0;
}

/* ---- tile store: 3x3 window of resident tiles ---- */
static void tilestore_flush_all(TileStore* ts) {
    bool all_ok=true;
    for(size_t i=0;i<TILE_CACHE;i++) {
        if(ts->slot[i] && ts->slot[i]->dirty) {
            if(!rtile_flush(ts->slot[i])) all_ok=false;
        }
    }
    ts->any_dirty=!all_ok;
    ts->last_flush_tick=furi_get_tick();
}

static RTile* tilestore_find(TileStore* ts, uint32_t tx, uint32_t ty) {
    for(size_t i=0;i<TILE_CACHE;i++) {
        if(ts->slot[i] && ts->slot[i]->tx==tx && ts->slot[i]->ty==ty) return ts->slot[i];
    }
    return NULL;
}

static bool tile_in_window(const RTile* tile, uint32_t ctx, uint32_t cty) {
    int32_t dx=(int32_t)tile->tx-(int32_t)ctx;
    int32_t dy=(int32_t)tile->ty-(int32_t)cty;
    return dx>=-1 && dx<=1 && dy>=-1 && dy<=1;
}

/* Make sure the 3x3 window around (ctx,cty) is resident, reusing slots that fell outside it. */
static void tilestore_set_center(TileStore* ts, uint32_t ctx, uint32_t cty) {
    if(ts->lock) furi_mutex_acquire(ts->lock,FuriWaitForever);
    ts->ctx=ctx;
    ts->cty=cty;
    ts->have_center=true;
    for(int dy=-1;dy<=1;dy++) {
        for(int dx=-1;dx<=1;dx++) {
            int32_t wx=(int32_t)ctx+dx;
            int32_t wy=(int32_t)cty+dy;
            if(wx<0 || wy<0) continue;
            if(tilestore_find(ts,(uint32_t)wx,(uint32_t)wy)) continue;

            int pick=-1;
            for(size_t i=0;i<TILE_CACHE;i++) {
                if(!ts->slot[i]) { pick=(int)i; break; }
            }
            if(pick<0) {
                for(size_t i=0;i<TILE_CACHE;i++) {
                    if(!tile_in_window(ts->slot[i],ctx,cty)) { pick=(int)i; break; }
                }
            }
            if(pick<0) continue;
            if(!ts->slot[pick]) {
                ts->slot[pick]=malloc(sizeof(RTile));
                if(!ts->slot[pick]) continue;
            } else if(ts->slot[pick]->dirty) {
                rtile_flush(ts->slot[pick]);
            }
            rtile_load(ts->slot[pick],(uint32_t)wx,(uint32_t)wy);
        }
    }
    if(ts->lock) furi_mutex_release(ts->lock);
}

/* Autoload: call on every fix. Only does work when you step into a different tile. */
static void tilestore_follow(TileStore* ts, double lat, double lon) {
    int32_t gx,gy;
    if(!global_xy(lat,lon,&gx,&gy)) return;
    uint32_t tx=(uint32_t)gx>>RT_TILE_SHIFT;
    uint32_t ty=(uint32_t)gy>>RT_TILE_SHIFT;
    if(!ts->have_center || tx!=ts->ctx || ty!=ts->cty) {
        tilestore_flush_all(ts); /* persist when leaving a tile */
        tilestore_set_center(ts,tx,ty);
    }
}

/* Does the segment touch the tile (grown by RT_MARGIN)? Coordinates are tile-local. */
static bool seg_touches_tile(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    float ax=(float)x0, ay=(float)y0, bx=(float)x1, by=(float)y1;
    return lb_clip(&ax,&ay,&bx,&by,
                   (float)-RT_MARGIN,(float)-RT_MARGIN,
                   (float)(RT_TILE_UNITS+RT_MARGIN),(float)(RT_TILE_UNITS+RT_MARGIN));
}

/* Store a segment (global units) in every resident tile it touches. */
static void rt_add_segment(TileStore* ts, int32_t x0, int32_t y0, int16_t a0,
                           int32_t x1, int32_t y1, int16_t a1, float thr_m) {
    int32_t minx=x0<x1 ? x0 : x1, maxx=x0<x1 ? x1 : x0;
    int32_t miny=y0<y1 ? y0 : y1, maxy=y0<y1 ? y1 : y0;
    int32_t lox=minx-RT_MARGIN, loy=miny-RT_MARGIN;
    if(lox<0) lox=0;
    if(loy<0) loy=0;
    int32_t tx0=lox>>RT_TILE_SHIFT, tx1=(maxx+RT_MARGIN)>>RT_TILE_SHIFT;
    int32_t ty0=loy>>RT_TILE_SHIFT, ty1=(maxy+RT_MARGIN)>>RT_TILE_SHIFT;
    float thr_units=thr_m/RT_UNIT_M;
    for(int32_t ty=ty0;ty<=ty1;ty++) {
        for(int32_t tx=tx0;tx<=tx1;tx++) {
            RTile* tile=tilestore_find(ts,(uint32_t)tx,(uint32_t)ty);
            if(!tile) continue;
            int32_t ox=tx<<RT_TILE_SHIFT, oy=ty<<RT_TILE_SHIFT;
            if(!seg_touches_tile(x0-ox,y0-oy,x1-ox,y1-oy)) continue;
            RPt p0={(int16_t)(x0-ox),(int16_t)(y0-oy),a0,(uint8_t)(a0!=ALT_UNKNOWN ? 1 : 0),0};
            RPt p1={(int16_t)(x1-ox),(int16_t)(y1-oy),a1,(uint8_t)(a1!=ALT_UNKNOWN ? 1 : 0),0};
            int added=rtile_add_segment(tile,p0,p1,thr_units,tile_kx((uint32_t)ty));
            if(tile->dirty) ts->any_dirty=true; /* altitude averaging counts as a change */
            if(added) {
                ts->any_dirty=true;
                /* count each stored segment once: in the tile that holds its end point */
                if((x1>>RT_TILE_SHIFT)==tx && (y1>>RT_TILE_SHIFT)==ty) ts->new_pts+=(uint32_t)added;
            }
        }
    }
}

static void tilestore_tick(TileStore* ts) {
    if(ts->any_dirty && (furi_get_tick()-ts->last_flush_tick)>=TILE_FLUSH_MS)
        tilestore_flush_all(ts);
}

/* Save what is dirty, then drop every resident tile. */
static void tilestore_reset(TileStore* ts) {
    tilestore_flush_all(ts);
    if(ts->lock) furi_mutex_acquire(ts->lock,FuriWaitForever);
    for(size_t i=0;i<TILE_CACHE;i++) {
        free(ts->slot[i]);
        ts->slot[i]=NULL;
    }
    ts->have_center=false;
    if(ts->lock) furi_mutex_release(ts->lock);
}

static void tilestore_free(TileStore* ts) { tilestore_reset(ts); }

typedef void (*SegFn)(void* ctx, int32_t x0, int32_t y0, int32_t x1, int32_t y1);

/* Visit every stored segment (global units) of the resident tiles,
   skipping tiles that are also resident in `skip` (may be NULL). */
static void tilestore_each_segment(TileStore* ts, TileStore* skip, SegFn fn, void* ctx) {
    for(size_t sl=0;sl<TILE_CACHE;sl++) {
        const RTile* tile=ts->slot[sl];
        if(!tile) continue;
        if(skip && tilestore_find(skip,tile->tx,tile->ty)) continue;
        int32_t ox=(int32_t)(tile->tx<<RT_TILE_SHIFT);
        int32_t oy=(int32_t)(tile->ty<<RT_TILE_SHIFT);
        for(int i=1;i<(int)tile->n;i++) {
            if(rt_is_break(&tile->p[i]) || rt_is_break(&tile->p[i-1])) continue;
            fn(ctx,ox+tile->p[i-1].x,oy+tile->p[i-1].y,ox+tile->p[i].x,oy+tile->p[i].y);
        }
    }
}

/* Record the point we just accepted: the segment from the previous one goes into the tiles. */
static void rt_record_point(Trail* t) {
    int32_t gx,gy;
    if(!global_xy(t->lat,t->lon,&gx,&gy)) return;
    int16_t alt=ALT_UNKNOWN;
    if(t->alt_smooth_ok) {
        int32_t a=(int32_t)roundf(t->alt_smooth*ALT_SCALE);
        if(a>32000) a=32000;
        if(a<-32000) a=-32000;
        alt=(int16_t)a;
    }
    tilestore_follow(&t->tiles,t->lat,t->lon);
    if(t->rt_have_last) {
        float kx=cosf(rad(t->lat));
        float dx=(float)(gx-t->rt_gx)*kx, dy=(float)(gy-t->rt_gy);
        float dm=sqrtf(dx*dx+dy*dy)*RT_UNIT_M;
        if(dm>=0.5f && dm<250.0f) {
            rt_add_segment(&t->tiles,t->rt_gx,t->rt_gy,t->rt_alt,gx,gy,alt,
                           merge_m[t->settings.merge_idx]);
        }
    }
    t->rt_gx=gx;
    t->rt_gy=gy;
    t->rt_alt=alt;
    t->rt_have_last=true;
}

/* Altitude of the stored route point nearest to a place (within max_m). */
static bool rt_nearest_alt(Trail* t, double lat, double lon, float max_m, int16_t* alt) {
    int32_t gx,gy;
    if(!global_xy(lat,lon,&gx,&gy)) return false;
    float kx=cosf(rad(lat));
    float best=max_m/RT_UNIT_M;
    best*=best;
    bool found=false;
    TileStore* stores[2]={&t->tiles,&t->view};
    for(int s=0;s<2;s++) {
        for(size_t sl=0;sl<TILE_CACHE;sl++) {
            const RTile* tile=stores[s]->slot[sl];
            if(!tile) continue;
            int32_t ox=(int32_t)(tile->tx<<RT_TILE_SHIFT);
            int32_t oy=(int32_t)(tile->ty<<RT_TILE_SHIFT);
            for(int i=0;i<(int)tile->n;i++) {
                if(rt_is_break(&tile->p[i])) continue;
                float dx=(float)(ox+tile->p[i].x-gx)*kx;
                float dy=(float)(oy+tile->p[i].y-gy);
                float d2=dx*dx+dy*dy;
                if(d2<=best) {
                    best=d2;
                    *alt=tile->p[i].alt;
                    found=true;
                }
            }
        }
    }
    return found;
}

/* ---- Free-move: tiles around the camera live in a second, lazily filled store ---- */
static void viewstore_follow(Trail* t) {
    int32_t gx,gy;
    if(!global_xy(t->cam_lat,t->cam_lon,&gx,&gy)) return;
    uint32_t tx=(uint32_t)gx>>RT_TILE_SHIFT;
    uint32_t ty=(uint32_t)gy>>RT_TILE_SHIFT;
    TileStore* v=&t->view;
    if(!v->have_center || tx!=v->ctx || ty!=v->cty) {
        tilestore_flush_all(&t->tiles); /* so what we read from the SD card is current */
        tilestore_set_center(v,tx,ty);
    }
}

static void viewstore_release(TileStore* v) { tilestore_reset(v); }
