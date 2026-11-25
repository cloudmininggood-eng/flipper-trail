/* flipper_trail module: cleanup routes. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
/* ---------------- Clean up routes: import old CSV recordings, merge repeats ---------------- */
typedef struct {
    File* f;
    uint8_t buf[256];
    uint16_t len, pos;
    bool eof;
} LineReader;

static bool lr_next(LineReader* lr, char* out, size_t cap) {
    size_t o=0;
    for(;;) {
        if(lr->pos>=lr->len) {
            if(lr->eof) { out[o]='\0'; return o>0; }
            lr->len=storage_file_read(lr->f,lr->buf,sizeof(lr->buf));
            lr->pos=0;
            if(lr->len==0) { lr->eof=true; out[o]='\0'; return o>0; }
        }
        char c=(char)lr->buf[lr->pos++];
        if(c=='\n') { out[o]='\0'; return true; }
        if(c!='\r' && o+1<cap) out[o++]=c;
    }
}

#define RT_IMPORTED_PATH RT_DIR "/imported.txt"

static char* imported_load(void) {
    char* buf=NULL;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,RT_IMPORTED_PATH,FSAM_READ,FSOM_OPEN_EXISTING)) {
        size_t size=(size_t)storage_file_size(f);
        if(size>8192) size=8192;
        buf=malloc(size+1);
        if(buf) {
            size_t got=size ? storage_file_read(f,buf,size) : 0;
            buf[got]='\0';
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    return buf;
}

static bool imported_has(const char* list, const char* name) {
    if(!list) return false;
    size_t nl=strlen(name);
    const char* p=list;
    while((p=strstr(p,name))!=NULL) {
        bool starts=(p==list || p[-1]=='\n');
        bool ends=(p[nl]=='\n' || p[nl]=='\0');
        if(starts && ends) return true;
        p+=nl;
    }
    return false;
}

static void imported_add(const char* name) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    storage_common_mkdir(storage,RT_DIR);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,RT_IMPORTED_PATH,FSAM_WRITE,FSOM_OPEN_APPEND)) {
        storage_file_write(f,name,strlen(name));
        storage_file_write(f,"\n",1);
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

static bool name_is_csv(const char* n) {
    size_t l=strlen(n);
    return l>4 && strcmp(n+l-4,".csv")==0;
}

/* Feed one recorded CSV into the route tiles through a scratch tile store. */
static void import_csv(Trail* t, TileStore* imp, const char* path) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,path,FSAM_READ,FSOM_OPEN_EXISTING)) {
        LineReader* lr=malloc(sizeof(LineReader));
        char* row=malloc(208);
        if(lr && row) {
            memset(lr,0,sizeof(LineReader));
            lr->f=f;
            bool have_prev=false;
            int32_t pgx=0, pgy=0;
            int16_t palt=ALT_UNKNOWN;
            float thr_m=merge_m[t->settings.merge_idx];
            while(lr_next(lr,row,208)) {
                if(row[0]=='l' || row[0]=='\0') continue; /* header or blank line */
                char alt_text[16]={0};
                int16_t alt=ALT_UNKNOWN;
                if(field(row,10,alt_text,sizeof(alt_text)) &&
                   ((alt_text[0]>='0' && alt_text[0]<='9') || alt_text[0]=='-' || alt_text[0]=='+')) {
                    int32_t a=(int32_t)roundf(parse_float(alt_text)*ALT_SCALE);
                    if(a>32000) a=32000;
                    if(a<-32000) a=-32000;
                    alt=(int16_t)a;
                }
                char* comma=strchr(row,',');
                if(!comma) continue;
                *comma++='\0';
                double lat=parse_double(row);
                double lon=parse_double(comma);
                int32_t gx,gy;
                if(!global_xy(lat,lon,&gx,&gy)) { have_prev=false; continue; }
                uint32_t tx=(uint32_t)gx>>RT_TILE_SHIFT;
                uint32_t ty=(uint32_t)gy>>RT_TILE_SHIFT;
                if(!imp->have_center || tx!=imp->ctx || ty!=imp->cty) {
                    tilestore_flush_all(imp);
                    tilestore_set_center(imp,tx,ty);
                }
                if(have_prev) {
                    float kx=cosf(rad(lat));
                    float dx=(float)(gx-pgx)*kx, dy=(float)(gy-pgy);
                    float dm=sqrtf(dx*dx+dy*dy)*RT_UNIT_M;
                    if(dm>=0.5f && dm<250.0f) rt_add_segment(imp,pgx,pgy,palt,gx,gy,alt,thr_m);
                }
                pgx=gx;
                pgy=gy;
                palt=alt;
                have_prev=true;
            }
            tilestore_flush_all(imp);
        }
        free(lr);
        free(row);
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

/* Tidy every tile file in one shard folder; adds the remaining point count to *pts. */
static void optimize_shard(Trail* t, RTile* tile, const char* dirpath, unsigned* pts) {
    float thr_m=merge_m[t->settings.merge_idx];
    if(thr_m<=0.0f) thr_m=RT_OPT_THR_M;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* d=storage_file_alloc(storage);
    if(storage_dir_open(d,dirpath)) {
        FileInfo info;
        char name[48];
        while(storage_dir_read(d,&info,name,sizeof(name))) {
            if(file_info_is_dir(&info)) continue;
            size_t l=strlen(name);
            if(l<5 || strcmp(name+l-4,".bin")!=0) continue;
            char path[128];
            snprintf(path,sizeof(path),"%s/%s",dirpath,name);
            if(!rtile_read(tile,path)) continue;
            uint16_t before=tile->n;
            rtile_compact(tile,RT_OPT_EPS_M/RT_UNIT_M,thr_m/RT_UNIT_M,tile_kx(tile->ty));
            if(tile->n!=before) rtile_write(tile,path);
            *pts+=tile->n;
        }
    }
    storage_dir_close(d);
    storage_file_free(d);
    furi_record_close(RECORD_STORAGE);
}

static void optimizer_run(Trail* t) {
    t->opt_state=0;
    if(t->mode==ModeRecording) {
        strlcpy(t->opt_text,"Stop recording first",sizeof(t->opt_text));
        return;
    }
    /* free RAM and make the SD card the single source of truth */
    tilestore_reset(&t->tiles);
    tilestore_reset(&t->view);

    unsigned files=0, pts=0;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    storage_common_mkdir(storage,RT_DIR);

    /* 1. import CSV recordings that are not in the route tiles yet */
    char* done=imported_load();
    TileStore* imp=malloc(sizeof(TileStore));
    if(imp) {
        memset(imp,0,sizeof(TileStore));
        File* dir=storage_file_alloc(storage);
        if(storage_dir_open(dir,ROUTES_DIR)) {
            FileInfo info;
            char name[64];
            while(storage_dir_read(dir,&info,name,sizeof(name))) {
                if(file_info_is_dir(&info) || !name_is_csv(name)) continue;
                if(imported_has(done,name)) continue;
                char path[112];
                snprintf(path,sizeof(path),ROUTES_DIR "/%s",name);
                import_csv(t,imp,path);
                imported_add(name);
                files++;
            }
        }
        storage_dir_close(dir);
        storage_file_free(dir);
        tilestore_free(imp);
        free(imp);
    }
    free(done);

    /* 2. tidy every tile: straighten, drop repeated paths */
    RTile* tile=malloc(sizeof(RTile));
    if(tile) {
        File* dir=storage_file_alloc(storage);
        if(storage_dir_open(dir,RT_DIR)) {
            FileInfo info;
            char name[32];
            while(storage_dir_read(dir,&info,name,sizeof(name))) {
                if(!file_info_is_dir(&info)) continue;
                char sub[64];
                snprintf(sub,sizeof(sub),RT_DIR "/%s",name);
                optimize_shard(t,tile,sub,&pts);
            }
        }
        storage_dir_close(dir);
        storage_file_free(dir);
        free(tile);
    }
    furi_record_close(RECORD_STORAGE);

    snprintf(t->opt_text,sizeof(t->opt_text),"%u csv, %u pts",files,pts);
    if(t->pos_known) tilestore_follow(&t->tiles,t->lat,t->lon); /* reload the window */
}

/* Hold OK on the map: pin a saved location at your position or (free-move) at the crosshair.
   At the crosshair, holding OK on an existing pin sends you there (or stops). */
static void map_pin(Trail* t, bool at_cursor) {
    double lat,lon;
    if(at_cursor) {
        lat=t->cam_lat;
        lon=t->cam_lon;
    } else if(t->pos_known) {
        lat=t->lat;
        lon=t->lon;
    } else {
        toast_set(t,"No position");
        return;
    }
    int near=wp_near(t,lat,lon,25.0f);
    char b[16];
    if(near>=0) {
        if(!at_cursor) {
            snprintf(b,sizeof(b),"At %s",t->wp[near].name);
            toast_set(t,b);
            return;
        }
        uint8_t id=t->wp[near].id;
        if(t->settings.dest_id==id) {
            dest_set(t,0);
            snprintf(b,sizeof(b),"Stopped");
        } else {
            dest_set(t,id);
            t->routing=true;
            snprintf(b,sizeof(b),"Go: %s",t->wp[near].name);
        }
        toast_set(t,b);
        return;
    }
    int idx=wp_add(t,lat,lon);
    if(idx<0) {
        toast_set(t,"List full (16)");
    } else {
        snprintf(b,sizeof(b),"Pinned %s",t->wp[idx].name);
        toast_set(t,b);
    }
}
