#include <furi.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/text_input.h>
#include <furi_hal.h>
#include <furi_hal_serial.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <gui/elements.h>
#include <input/input.h>
#include <storage/storage.h>
#include <expansion/expansion.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* First-stage prototype. Standard Dr. Where default: 9600 baud.
   For a Pro unit, change TRAIL_BAUD to 115200. */
#define TRAIL_BAUD 9600
#define TRAIL_LINE_MAX 128
#define ROUTES_DIR "/ext/GPS/routes"
#define EARTH_M 6371000.0f
#define EVENT_QUEUE_LEN 32
#define UI_BODY_TOP 12
#define UI_BODY_BOTTOM 50 /* header = y0..10, footer buttons = y52..63 */

typedef enum { ModeHome, ModeRecording, ModeRetrace } TrailMode;
typedef enum { TrailTabRoute=0, TrailTabMap, TrailTabStats, TrailTabTime, TrailTabElev, TrailTabCount } TrailTab;
typedef enum { HomePageRoute=0, HomePageMap, HomePageMenu, HomePageStats, HomePageTime, HomePageCount } TrailHomePage;
typedef enum { MapViewFit=0, MapViewFollow, MapViewCount } MapViewMode;
typedef enum { MapScreenWide=0, MapScreenTall, MapScreenTallFlip, MapScreenCount } MapScreenMode;
typedef enum { MapLayerRoute=0, MapLayerSaved, MapLayerBoth, MapLayerCount } MapLayerMode;
typedef enum { FacingNorth=0, FacingHeading, FacingRoute, FacingCount } FacingMode;
typedef enum {
    SettingsDestination=0, SettingsMapView, SettingsMapFacing, SettingsMapScreen, SettingsMapLayer, SettingsZoom, SettingsFreeMove,
    SettingsMoveStep, SettingsMerge, SettingsLink, SettingsOptimize, SettingsIdle, SettingsUnits,
    SettingsTimeZone, SettingsTrim, SettingsTrimStep, SettingsClock, SettingsBaud, SettingsPageCount
} SettingsPage;
typedef struct {
    uint8_t map_view;   /* MapViewMode */
    uint8_t map_layer;  /* MapLayerMode */
    uint8_t zoom;       /* index into zoom_span_m[] (follow view) */
    uint8_t free_move;  /* 1 = OK on the map starts free-move */
    uint8_t imperial;   /* 0 metric, 1 imperial */
    uint8_t baud_idx;   /* index into baud_values[] */
    uint8_t tz_auto;    /* 1 = work the time zone out from GPS position */
    int8_t tz_half_hours; /* manual UTC offset in 30-minute steps (-24..+28) */
    uint8_t merge_idx;  /* 0 off, 1..3 = how close a repeated path must be to be dropped */
    uint8_t move_idx;   /* free-move step: 0 = 20 px, else index into move_m[] */
    uint8_t link_idx;   /* routing: index into link_m[] */
    uint8_t dest_id;    /* id of the destination waypoint, 0 = none */
    uint8_t facing;     /* FacingMode: north up, heading up, route up */
    uint8_t idle_idx;   /* index into idle_s[]: stop saving points after standing still this long */
    uint8_t sync_clock; /* 1 = set the Flipper's own clock from GPS (local time) */
    uint8_t map_screen; /* MapScreenMode: map on the wide screen, or held upright (64x128) */
    int16_t trim_ms;    /* manual time offset in ms: + shows the clock later, - earlier */
    uint8_t trim_step_idx; /* index into trim_steps[]: size of one trim tap */
} TrailSettings;

/* ---- Auto time zone: /ext/GPS/tz.bin, made on a PC by gen_tz.py ----
   12-byte header | zone_count x TzZone | rows*cols grid bytes (zone index per cell). */
#define TZ_PATH "/ext/GPS/tz.bin"
#define TZ_MAGIC "FTZ"
#define TZ_VERSION 1
#define TZ_MAX_TRANS 32
typedef struct {
    char magic[3];
    uint8_t version;
    uint16_t cols, rows, zone_count, reserved;
} TzHeader;
typedef struct {
    int16_t std_min;          /* UTC offset (minutes) before the first transition */
    uint8_t n;                /* number of transitions */
    uint8_t pad;
    uint32_t t[TZ_MAX_TRANS]; /* transition instants, UTC epoch seconds, ascending */
    int16_t off[TZ_MAX_TRANS];/* UTC offset (minutes) in force from that instant on */
} TzZone;
typedef enum { TzStateUnknown=0, TzStateOk, TzStateMissing } TzState;
typedef struct {
    TzState state;
    uint32_t retry_tick;
    TzHeader h;
    int col, row;
    bool zone_ok;
    TzZone zone;
} TzCache;
typedef struct { double lat, lon; float alt; } Point; /* alt: smoothed metres, NAN if unknown */

/* ---- Route tiles: saved routes, stored per map tile ----
   Position unit = 1/64 of a cell (cell = 0.0001 deg: ~11 m N-S, ~9 m E-W at mid latitudes).
   Tile = 32x32 cells = 2048 units (~350 x 290 m). One small file per tile holds the route
   segments that touch it, so only the tile you are in and its 8 neighbours are in RAM. */
#define TILE_BITS 5
#define TILE_DIM (1<<TILE_BITS)
#define TILE_CACHE 9
#define CELLS_PER_DEG 10000.0f
#define RT_UNITS 64
#define RT_TILE_SHIFT 11
#define RT_TILE_UNITS (1<<RT_TILE_SHIFT)
#define RT_MARGIN 64          /* a segment within ~1 cell of a tile is stored in that tile too */
#define RT_MAX 320            /* points per tile */
#define RT_UNIT_M 0.173938f   /* metres per unit, north-south */
#define RT_DIR "/ext/GPS/rtiles"
#define RT_MAGIC "FTR"
#define RT_VERSION 2
#define RT_BREAK ((int16_t)-32768) /* x == RT_BREAK ends a stroke */
#define TILE_FLUSH_MS 10000
#define ALT_UNKNOWN ((int16_t)-32768)
#define ALT_SCALE 2.0f /* stored altitude units per metre (0.5 m steps) */
#define MERGE_COUNT 4
#define MOVE_COUNT 6          /* free-move step choices */
#define LINK_COUNT 4          /* routing: max gap between dots that may be joined */
#define WP_MAX 16             /* saved locations */
#define WP_PATH "/ext/GPS/waypoints.bin"
#define NAV_NODES 900         /* route points considered when routing */
#define NAV_EDGES 1800
#define NAV_PATH_MAX 160      /* points kept of the computed route */
#define RT_OPT_EPS_M 1.5f     /* straightening tolerance used when tidying a tile */
#define RT_OPT_THR_M 8.0f     /* "same path" distance used by Clean up when merging is off */

/* tile-local position (units from tile origin), altitude, and how many altitude samples
   that altitude is the average of (0 = altitude unknown) */
typedef struct { int16_t x, y, alt; uint8_t n, pad; } RPt;
typedef struct { int16_t x, y, alt; } RPtV1; /* version 1 files */

typedef struct {
    char magic[3];
    uint8_t version;
    uint16_t n;
    uint16_t reserved;
    uint32_t tx, ty;
} RTileHeader; /* 16 bytes, followed by n x 8-byte points (version 1: 6-byte points) */

typedef struct {
    uint32_t tx, ty;
    bool dirty;
    uint16_t n;
    RPt p[RT_MAX];
} RTile;

typedef struct {
    RTile* slot[TILE_CACHE];
    uint32_t ctx, cty;
    bool have_center;
    bool any_dirty;
    uint32_t last_flush_tick;
    uint32_t new_pts;   /* route points stored since recording started */
    FuriMutex* lock;    /* shared with the GUI thread; held while the window is reloaded */
} TileStore;

typedef struct { double lat, lon; uint8_t id; char name[7]; uint16_t radius_m; } Waypoint; /* in RAM; radius 0 = a point, else an area */
typedef struct { int32_t lat_e7, lon_e7; uint8_t id; char name[7]; } WpRec; /* on the SD card, 1e-7 degree */
typedef struct { float lat, lon; uint8_t id; char name[7]; } WpRecV1; /* version 1 files */
typedef struct { int32_t lat_e7, lon_e7; uint8_t id; char name[7]; uint16_t radius_m; uint16_t pad; } WpRec3; /* version 3 files */
typedef struct {
    char magic[3];
    uint8_t version;
    uint8_t count;
    uint8_t next_id;
    uint16_t reserved;
} WpHeader;
typedef struct { int32_t x, y; } GPt; /* global position, units */
typedef struct {
    FuriMessageQueue* rx_queue;
    FuriMutex* mutex; /* guards route[]/route_count between app thread and GUI thread */
    char line[TRAIL_LINE_MAX];
    size_t line_len;
    bool fix;
    double lat, lon;
    double prev_lat, prev_lon;
    bool have_prev;
    double sample_prev_lat, sample_prev_lon;
    bool have_sample_prev;
    char timestamp[32];
    float speed_mps;
    bool speed_valid;
    int satellites;
    float hdop;
    float altitude_m;
    bool altitude_valid;
    float alt_smooth;   /* low-pass copy used for the elevation graph */
    bool alt_smooth_ok;
    int fix_quality;
    uint8_t slow_speed_samples;
    uint8_t fast_speed_samples;
    double distance;
    uint32_t points;
    TrailMode mode;
    TrailTab active_tab;
    TrailHomePage home_page;
    char route_path[96];
    char status[24];
    Point route[300];
    size_t route_count;
    size_t target_index;
    TileStore tiles;
    TrailSettings settings;
    bool settings_open;
    uint8_t settings_page;
    bool utc_valid;       /* a plausible UTC date+time has been received */
    uint32_t utc_epoch;   /* UTC epoch seconds at the last RMC sentence */
    uint32_t utc_tick;    /* furi_get_tick() at the start of the UTC second utc_epoch (filtered) */
    uint32_t utc_sync_tick; /* furi_get_tick() when the last new GPS second arrived */
    uint32_t line_start_tick; /* when the sentence being parsed began arriving */
    uint32_t clock_sync_tick; /* when the Flipper clock was last set from GPS */
    uint32_t clock_check_tick;
    bool clock_synced, clock_pending;
    uint8_t area_in_id; /* id of the area you are inside, 0 = none */
    volatile uint32_t nl_tick[32]; /* tick of each received line end, filled by the UART callback */
    volatile uint8_t nl_head, nl_tail;
    bool pos_known;       /* had a fix at least once (lat/lon usable for tz) */
    int16_t tz_offset_min;
    bool tz_known;
    bool tz_solar;        /* auto fell back to longitude/15 (no tz.bin) */
    uint32_t tz_last_tick;
    TzCache tz;
    bool map_free;        /* free-move mode on the map */
    double cam_lat, cam_lon; /* free-move camera centre */
    TileStore view;       /* tiles around the camera (allocated only while moving) */
    bool saved_pos_valid;
    double saved_lat, saved_lon;
    uint32_t lastpos_tick;
    int32_t rt_gx, rt_gy;   /* previous recorded point, global units */
    int16_t rt_alt;
    bool rt_have_last;
    uint8_t opt_state;      /* 0 idle, 1 show 'working', 2 run now */
    char opt_text[24];
    Waypoint wp[WP_MAX];
    uint8_t wp_count;
    uint8_t wp_next_id;
    GPt nav_path[NAV_PATH_MAX]; /* computed route from you to the destination */
    uint16_t nav_n;
    uint8_t nav_state;      /* 0 none, 1 route found, 2 no route, 3 too far / too many points */
    float nav_route_m;
    float nav_direct_m;
    int32_t nav_from_gx, nav_from_gy;
    uint32_t nav_tick;
    bool nav_dirty;
    char toast[16];
    uint32_t toast_until;
    bool routing;           /* guidance on: route computed, Route page shows only the way there */
    uint8_t route_row;      /* Route page: focused row */
    uint8_t route_sel;      /* Route page: selected saved location (index) */
    bool route_edit;        /* keyboard open (rename in progress) */
    char rename_buf[7];     /* text being edited: up to 6 characters */
    volatile uint8_t rename_state; /* 1 editing, 2 saved, 3 cancelled */
    FuriThread* rename_thread;
    uint8_t edit_wp;
    float hdg_sin, hdg_cos; /* smoothed GPS course (unit vector) */
    bool hdg_valid;
    uint32_t last_move_tick; /* last time sustained movement was seen (idle stop) */
    bool idle;               /* standing still long enough that new points are not saved */
} Trail;
typedef struct { InputEvent event; } UiEvent;

typedef struct { uint8_t byte; } RxByte;

static float parse_float(const char* s) {
    if(!s) return 0.0f;
    bool negative=false;
    if(*s=='-' || *s=='+') { negative=(*s=='-'); s++; }
    float value=0.0f;
    while(*s>='0' && *s<='9') { value=value*10.0f+(float)(*s-'0'); s++; }
    if(*s=='.') {
        s++;
        float place=0.1f;
        while(*s>='0' && *s<='9') {
            value+=(float)(*s-'0')*place;
            place*=0.1f;
            s++;
        }
    }
    return negative ? -value : value;
}

/* Absolute positions are doubles: a float only resolves about 1 m at these longitudes. */
static double parse_double(const char* s) {
    if(!s) return 0.0;
    bool negative=false;
    if(*s=='-' || *s=='+') { negative=(*s=='-'); s++; }
    double mant=0.0;
    int frac=0;
    bool in_frac=false;
    for(;*s;s++) {
        if(*s>='0' && *s<='9') {
            if(mant<(double)9.0e15) {
                mant=mant*(double)10.0+(double)(*s-'0');
                if(in_frac) frac++;
            }
        } else if(*s=='.' && !in_frac) {
            in_frac=true;
        } else {
            break;
        }
    }
    double div=1.0;
    for(int i=0;i<frac;i++) div*=(double)10.0;
    double v=mant/div;
    return negative ? -v : v;
}
static int32_t deg_to_e7(double d) { return (int32_t)floor(d*(double)1e7+(double)0.5); }
static double e7_to_deg(int32_t v) { return (double)v/(double)1e7; }

static float rad(float d) { return d * 0.017453292519943295f; }
static float dist_m(double a, double b, double c, double d) {
    float p1=rad((float)a), p2=rad((float)c), dp=rad((float)(c-a)), dl=rad((float)(d-b));
    float sdp=sinf(dp/2.0f), sdl=sinf(dl/2.0f);
    float x=sdp*sdp+cosf(p1)*cosf(p2)*sdl*sdl;
    if(x>1.0f) x=1.0f;
    if(x<0.0f) x=0.0f;
    return EARTH_M*2.0f*atan2f(sqrtf(x),sqrtf(1.0f-x));
}

#define IDLE_COUNT 4
static const uint8_t idle_s[IDLE_COUNT]={0,10,20,60}; /* seconds, 0 = off */
static const float merge_m[MERGE_COUNT]={0.0f,4.0f,8.0f,15.0f};
#define AREA_R_COUNT 7
static const uint16_t area_r[AREA_R_COUNT]={0,25,50,100,200,400,800}; /* metres, 0 = plain point */
static const float move_m[MOVE_COUNT]={0.0f,10.0f,25.0f,50.0f,100.0f,250.0f};
static const float link_m[LINK_COUNT]={6.0f,12.0f,25.0f,50.0f};

static bool global_xy(double lat, double lon, int32_t* gx, int32_t* gy) {
    if(lat<(double)-90.0 || lat>(double)90.0 || lon<(double)-180.0 || lon>(double)180.0) return false;
    double x=(lon+(double)180.0)*(double)CELLS_PER_DEG*(double)RT_UNITS;
    double y=(lat+(double)90.0)*(double)CELLS_PER_DEG*(double)RT_UNITS;
    *gx=(int32_t)floor(x);
    *gy=(int32_t)floor(y);
    return true;
}

static float tile_kx(uint32_t ty) { /* east-west shrink factor at this tile's latitude */
    float lat=((float)ty+0.5f)*(float)RT_TILE_UNITS/(CELLS_PER_DEG*(float)RT_UNITS)-90.0f;
    return cosf(rad(lat));
}

/* Liang-Barsky: clip a segment to a rectangle. False if nothing of it is inside. */
static bool lb_clip(float* x0, float* y0, float* x1, float* y1,
                    float xmin, float ymin, float xmax, float ymax) {
    float dx=*x1-*x0, dy=*y1-*y0;
    float p[4]={-dx,dx,-dy,dy};
    float q[4]={*x0-xmin,xmax-*x0,*y0-ymin,ymax-*y0};
    float u0=0.0f, u1=1.0f;
    for(int i=0;i<4;i++) {
        if(fabsf(p[i])<1e-6f) {
            if(q[i]<0.0f) return false;
        } else {
            float r=q[i]/p[i];
            if(p[i]<0.0f) {
                if(r>u1) return false;
                if(r>u0) u0=r;
            } else {
                if(r<u0) return false;
                if(r<u1) u1=r;
            }
        }
    }
    float nx0=*x0+u0*dx, ny0=*y0+u0*dy, nx1=*x0+u1*dx, ny1=*y0+u1*dy;
    *x0=nx0; *y0=ny0; *x1=nx1; *y1=ny1;
    return true;
}

static void rt_path(char* out, size_t cap, uint32_t tx, uint32_t ty, bool dir_only) {
    if(dir_only)
        snprintf(out,cap,RT_DIR "/%lu_%lu",(unsigned long)(tx>>5),(unsigned long)(ty>>5));
    else
        snprintf(out,cap,RT_DIR "/%lu_%lu/%lu_%lu.bin",
                 (unsigned long)(tx>>5),(unsigned long)(ty>>5),
                 (unsigned long)tx,(unsigned long)ty);
}

static void rtile_reset(RTile* tile, uint32_t tx, uint32_t ty) {
    tile->tx=tx;
    tile->ty=ty;
    tile->dirty=false;
    tile->n=0;
}

static bool rtile_read(RTile* tile, const char* path) {
    bool ok=false;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    File* f=storage_file_alloc(storage);
    if(storage_file_open(f,path,FSAM_READ,FSOM_OPEN_EXISTING)) {
        RTileHeader h;
        if(storage_file_read(f,&h,sizeof(h))==sizeof(h) &&
           memcmp(h.magic,RT_MAGIC,3)==0 && (h.version==RT_VERSION || h.version==1)) {
            if(h.version==RT_VERSION) {
                size_t body=(size_t)h.n*sizeof(RPt);
                if(h.n<=RT_MAX && (body==0 || storage_file_read(f,tile->p,body)==body)) {
                    tile->tx=h.tx;
                    tile->ty=h.ty;
                    tile->n=h.n;
                    tile->dirty=false;
                    ok=true;
                }
            } else {
                /* version 1: 6-byte points without a sample count; convert, keep what fits */
                uint16_t total=h.n<RT_MAX ? h.n : RT_MAX;
                uint16_t done=0;
                RPtV1 buf[24];
                ok=true;
                while(done<total && ok) {
                    uint16_t chunk=(uint16_t)((total-done)<24 ? (total-done) : 24);
                    if(storage_file_read(f,buf,(uint16_t)(chunk*sizeof(RPtV1)))!=chunk*sizeof(RPtV1)) {
                        ok=false;
                        break;
                    }
                    for(uint16_t i=0;i<chunk;i++) {
                        RPt* q=&tile->p[done+i];
                        q->x=buf[i].x;
                        q->y=buf[i].y;
                        q->alt=buf[i].alt;
                        q->n=(buf[i].x!=RT_BREAK && buf[i].alt!=ALT_UNKNOWN) ? 1 : 0;
                        q->pad=0;
                    }
                    done=(uint16_t)(done+chunk);
                }
                if(ok) {
                    tile->tx=h.tx;
                    tile->ty=h.ty;
                    tile->n=total;
                    tile->dirty=false;
                }
            }
        }
    }
    storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

static void rtile_load(RTile* tile, uint32_t tx, uint32_t ty) {
    char path[64];
    rt_path(path,sizeof(path),tx,ty,false);
    rtile_reset(tile,tx,ty);
    if(!rtile_read(tile,path) || tile->tx!=tx || tile->ty!=ty) rtile_reset(tile,tx,ty);
}

/* Writes the tile; an empty tile removes its file. */
static bool rtile_write(const RTile* tile, const char* path) {
    bool ok=false;
    Storage* storage=furi_record_open(RECORD_STORAGE);
    if(tile->n==0) {
        storage_common_remove(storage,path);
        ok=true;
    } else {
        RTileHeader h={.magic={'F','T','R'},.version=RT_VERSION,.n=tile->n,.reserved=0,
                       .tx=tile->tx,.ty=tile->ty};
        File* f=storage_file_alloc(storage);
        if(storage_file_open(f,path,FSAM_WRITE,FSOM_CREATE_ALWAYS)) {
            size_t body=(size_t)tile->n*sizeof(RPt);
            ok=storage_file_write(f,&h,sizeof(h))==sizeof(h) &&
               storage_file_write(f,tile->p,body)==body;
        }
        storage_file_close(f);
        storage_file_free(f);
    }
    furi_record_close(RECORD_STORAGE);
    return ok;
}

static bool rtile_flush(RTile* tile) {
    if(!tile->dirty) return true;
    char dir[48], path[64];
    rt_path(dir,sizeof(dir),tile->tx,tile->ty,true);
    rt_path(path,sizeof(path),tile->tx,tile->ty,false);
    Storage* storage=furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage,"/ext/GPS");
    storage_common_mkdir(storage,RT_DIR);
    storage_common_mkdir(storage,dir);
    furi_record_close(RECORD_STORAGE);
    bool ok=rtile_write(tile,path);
    if(ok) tile->dirty=false;
    return ok;
}

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

/* Keep the route to the destination fresh: on changes, after moving 40 m, or every minute. */
/* Toast when you step into or out of an area. Leaving needs 8 m of margin so GPS jitter at
   the edge does not flicker. */
static void area_update(Trail* t) {
    if(!t->pos_known || !t->fix) return;
    int cur=-1;
    float cur_d=0.0f;
    int best=-1;
    float best_r=0.0f;
    for(int i=0;i<(int)t->wp_count;i++) {
        const Waypoint* w=&t->wp[i];
        if(w->radius_m==0) continue;
        float e,n;
        map_local_xy(w->lat,w->lon,t->lat,t->lon,&e,&n);
        float d=sqrtf(e*e+n*n);
        if(w->id==t->area_in_id) {
            cur=i;
            cur_d=d;
        }
        if(d<=(float)w->radius_m && (best<0 || (float)w->radius_m<best_r)) {
            best=i;
            best_r=(float)w->radius_m;
        }
    }
    if(t->area_in_id!=0) {
        if(cur>=0 && cur_d<=(float)t->wp[cur].radius_m+8.0f) return; /* still inside */
        char b[24];
        if(cur>=0) snprintf(b,sizeof(b),"Left %s",t->wp[cur].name);
        else snprintf(b,sizeof(b),"Left area");
        t->area_in_id=0;
        toast_set(t,b);
    }
    if(best>=0) {
        char b[24];
        t->area_in_id=t->wp[best].id;
        snprintf(b,sizeof(b),"In %s",t->wp[best].name);
        toast_set(t,b);
    }
}

static void nav_tick(Trail* t) {
    if(t->settings.dest_id==0 || !t->routing) {
        if(t->nav_state!=0) nav_set_state(t,0);
        return;
    }
    if(!t->pos_known || t->settings_open || t->opt_state!=0) return;
    uint32_t now=furi_get_tick();
    bool need=t->nav_dirty;
    if(!need && t->nav_tick!=0) {
        uint32_t age=now-t->nav_tick;
        if(age>=60000) need=true;
        if(!need && age>=15000) {
            int32_t gx,gy;
            if(global_xy(t->lat,t->lon,&gx,&gy)) {
                float kx=cosf(rad(t->lat));
                float dx=(float)(gx-t->nav_from_gx)*kx, dy=(float)(gy-t->nav_from_gy);
                if(sqrtf(dx*dx+dy*dy)*RT_UNIT_M>=40.0f) need=true;
            }
        }
    } else if(t->nav_tick==0) {
        need=true;
    }
    if(need) nav_compute(t);
}

/* Rename uses the stock Flipper keyboard. It runs in its own thread with its own ViewDispatcher so
   GPS parsing and recording carry on underneath while the keyboard is open. */
typedef struct {
    Trail* t;
    ViewDispatcher* vd;
} RenameCtx;

static void rename_done_cb(void* context) {
    RenameCtx* c=context;
    c->t->rename_state=2;
    view_dispatcher_stop(c->vd);
}

static bool rename_back_cb(void* context) {
    RenameCtx* c=context;
    c->t->rename_state=3;
    return false; /* not handled here: the dispatcher stops itself */
}

static int32_t rename_worker(void* context) {
    Trail* t=context;
    RenameCtx c={.t=t,.vd=NULL};
    c.vd=view_dispatcher_alloc();
    TextInput* ti=text_input_alloc();
    view_dispatcher_set_event_callback_context(c.vd,&c);
    view_dispatcher_set_navigation_event_callback(c.vd,rename_back_cb);
    view_dispatcher_add_view(c.vd,0,text_input_get_view(ti));
    text_input_set_header_text(ti,"Rename point");
    text_input_set_result_callback(ti,rename_done_cb,&c,t->rename_buf,sizeof(t->rename_buf),false);
    Gui* gui=furi_record_open(RECORD_GUI);
    view_dispatcher_attach_to_gui(c.vd,gui,ViewDispatcherTypeFullscreen);
    view_dispatcher_switch_to_view(c.vd,0);
    view_dispatcher_run(c.vd);
    view_dispatcher_remove_view(c.vd,0);
    text_input_free(ti);
    view_dispatcher_free(c.vd);
    furi_record_close(RECORD_GUI);
    return 0;
}

static void route_edit_begin(Trail* t, int idx) {
    if(t->rename_thread) return;
    memset(t->rename_buf,0,sizeof(t->rename_buf));
    strlcpy(t->rename_buf,t->wp[idx].name,sizeof(t->rename_buf));
    t->edit_wp=t->wp[idx].id; /* the point's id, so a changed list can't rename the wrong one */
    t->rename_state=1;
    t->rename_thread=furi_thread_alloc_ex("TrailRename",4096,rename_worker,t);
    furi_thread_start(t->rename_thread);
    t->route_edit=true;
}

static void route_edit_commit(Trail* t) {
    Waypoint* w=NULL;
    for(uint8_t i=0;i<t->wp_count;i++) {
        if(t->wp[i].id==t->edit_wp) { w=&t->wp[i]; break; }
    }
    if(!w) return;
    char name[7];
    strlcpy(name,t->rename_buf,sizeof(name));
    size_t end=strlen(name);
    while(end>0 && name[end-1]==' ') name[--end]='\0';
    const char* start=name;
    while(*start==' ') start++;
    if(*start=='\0') snprintf(w->name,sizeof(w->name),"WP%u",(unsigned)w->id);
    else snprintf(w->name,sizeof(w->name),"%s",start);
    wp_save(t);
    toast_set(t,"Renamed");
}

/* Called from the main loop: pick up the result once the keyboard has closed. */
static void route_edit_poll(Trail* t) {
    if(!t->rename_thread) return;
    if(furi_thread_get_state(t->rename_thread)!=FuriThreadStateStopped) return;
    furi_thread_join(t->rename_thread);
    furi_thread_free(t->rename_thread);
    t->rename_thread=NULL;
    t->route_edit=false;
    if(t->rename_state==2) route_edit_commit(t);
}

/* Up/Down/OK on the Route page (Left/Right/Back keep their usual page meaning). */
static void route_page_input(Trail* t, const InputEvent* e) {
    bool is_short=(e->type==InputTypeShort);
    if(t->routing) {
        if(e->key==InputKeyOk && is_short) {
            t->routing=false;
            toast_set(t,"Routing off");
        }
        return;
    }
    if(e->key==InputKeyUp) {
        if(t->route_row>0) t->route_row--;
        return;
    }
    if(e->key==InputKeyDown) {
        if(t->route_row<ROUTE_ROWS-1) t->route_row++;
        return;
    }
    if(e->key!=InputKeyOk) return;
    int sel=route_sel_idx(t);
    if(e->type==InputTypeLong) {
        if(t->route_row==4 && sel>=0) {
            wp_delete(t,sel);
            settings_save(&t->settings);
            toast_set(t,"Deleted");
        }
        return;
    }
    if(!is_short) return;
    switch(t->route_row) {
    case 0:
        if(t->wp_count>0) t->route_sel=(uint8_t)((t->route_sel+1)%t->wp_count);
        break;
    case 1:
        if(sel<0) {
            toast_set(t,"No points yet");
        } else if(t->settings.dest_id==t->wp[sel].id) {
            dest_set(t,0);
            toast_set(t,"Cleared");
        } else {
            dest_set(t,t->wp[sel].id);
            t->routing=true; /* choosing a destination here starts navigating right away */
            toast_set(t,"Routing on");
        }
        break;
    case 2:
        if(sel>=0) route_edit_begin(t,sel);
        else toast_set(t,"No points yet");
        break;
    case 3:
        if(sel<0) {
            toast_set(t,"No points yet");
        } else {
            /* OK cycles: plain point, then areas of growing radius */
            Waypoint* aw=&t->wp[sel];
            int ri=0;
            for(int i=0;i<AREA_R_COUNT;i++) {
                if(area_r[i]==aw->radius_m) ri=i;
            }
            aw->radius_m=area_r[(ri+1)%AREA_R_COUNT];
            wp_save(t);
            t->nav_dirty=true;
            toast_set(t,aw->radius_m>0 ? "Area" : "Point");
        }
        break;
    case 4:
        toast_set(t,"Hold OK to delete");
        break;
    case 5: {
        uint8_t before=t->wp_count;
        map_pin(t,false);
        if(t->wp_count>before) t->route_sel=(uint8_t)(t->wp_count-1);
        break;
    }
    case 6:
        if(t->settings.dest_id==0 && sel>=0) { /* nothing chosen yet: use the selected point */
            dest_set(t,t->wp[sel].id);
        }
        if(wp_index_by_id(t,t->settings.dest_id)<0) {
            toast_set(t,"Pick a point");
        } else {
            t->routing=true;
            t->nav_dirty=true;
        }
        break;
    default:
        t->settings.link_idx=(uint8_t)((t->settings.link_idx+1)%LINK_COUNT);
        settings_save(&t->settings);
        t->nav_dirty=true;
        break;
    }
}

static void map_free_enter(Trail* t) {
    if(t->pos_known) {
        t->cam_lat=t->lat;
        t->cam_lon=t->lon;
    } else if(t->route_count>0) {
        t->cam_lat=t->route[t->route_count-1].lat;
        t->cam_lon=t->route[t->route_count-1].lon;
    } else {
        return; /* nothing to look at yet */
    }
    t->map_free=true;
    viewstore_follow(t);
}

static void map_free_exit(Trail* t) {
    t->map_free=false;
    viewstore_release(&t->view);
}

/* One pan step = 20 pixels at the current zoom. */
static void map_pan(Trail* t, InputKey key) {
    float scale=124.0f/zoom_span_m[t->settings.zoom];
    float step=t->settings.move_idx ? move_m[t->settings.move_idx] : 20.0f/scale;
    float coslat=cosf(rad(t->cam_lat));
    if(coslat<0.05f) coslat=0.05f;
    float dlat=step/111320.0f;
    float dlon=step/(111320.0f*coslat);
    switch(key) {
    case InputKeyUp: t->cam_lat+=(double)dlat; break;
    case InputKeyDown: t->cam_lat-=(double)dlat; break;
    case InputKeyLeft: t->cam_lon-=(double)dlon; break;
    case InputKeyRight: t->cam_lon+=(double)dlon; break;
    default: return;
    }
    if(t->cam_lat>(double)85.0f) t->cam_lat=(double)85.0f;
    if(t->cam_lat<(double)-85.0f) t->cam_lat=(double)-85.0f;
    if(t->cam_lon>(double)179.99f) t->cam_lon=(double)179.99f;
    if(t->cam_lon<(double)-179.99f) t->cam_lon=(double)-179.99f;
    viewstore_follow(t);
}

int32_t flipper_trail_app(void* p) {
    UNUSED(p);
    Trail* t=malloc(sizeof(Trail)); /* heap: route[] makes Trail large for the app stack */
    if(!t) return -1;
    memset(t,0,sizeof(Trail));
    t->mode=ModeHome; t->home_page=HomePageMenu; t->satellites=-1; t->hdop=-1.0f; t->fix_quality=-1;
    strlcpy(t->status,"WAITING",sizeof(t->status));
    settings_load(&t->settings);
    t->mutex=furi_mutex_alloc(FuriMutexTypeNormal);
    t->tiles.lock=t->mutex;
    t->view.lock=t->mutex;
    wp_load(t);
    {
        /* last known position: the map and auto time zone work before the first fix */
        double lp_lat,lp_lon;
        if(lastpos_load(&lp_lat,&lp_lon)) {
            t->lat=lp_lat;
            t->lon=lp_lon;
            t->pos_known=true;
            t->saved_pos_valid=true;
            t->saved_lat=lp_lat;
            t->saved_lon=lp_lon;
            tilestore_follow(&t->tiles,lp_lat,lp_lon);
        }
    }
    t->rx_queue=furi_message_queue_alloc(1024,sizeof(RxByte));
    Expansion* expansion=furi_record_open(RECORD_EXPANSION);
    expansion_disable(expansion); /* release expansion UART before acquiring it */
    FuriHalSerialHandle* serial=furi_hal_serial_control_acquire(FuriHalSerialIdUsart);
    if(!serial) {
        expansion_enable(expansion); furi_record_close(RECORD_EXPANSION);
        furi_message_queue_free(t->rx_queue); furi_mutex_free(t->mutex); free(t);
        return -1;
    }
    furi_hal_serial_init(serial,baud_values[t->settings.baud_idx]);
    furi_hal_serial_async_rx_start(serial,rx_callback,t,false);
    FuriMessageQueue* uiq=furi_message_queue_alloc(EVENT_QUEUE_LEN,sizeof(UiEvent));
    ViewPort* vp=view_port_alloc(); view_port_draw_callback_set(vp,draw,t); view_port_input_callback_set(vp,input_cb,uiq);
    Gui* gui=furi_record_open(RECORD_GUI); gui_add_view_port(gui,vp,GuiLayerFullscreen);
    bool running=true;
    while(running) {
        RxByte rb;
        while(furi_message_queue_get(t->rx_queue,&rb,0)==FuriStatusOk) {
            char c=(char)rb.byte;
            if(c=='\n') {
                uint32_t lt=furi_get_tick();
                if(t->nl_tail!=t->nl_head) {
                    lt=t->nl_tick[t->nl_tail];
                    t->nl_tail=(uint8_t)((t->nl_tail+1)&31);
                }
                /* the sentence started this long before its last byte arrived */
                uint32_t tx_ms=((uint32_t)t->line_len+2u)*10000u/baud_values[t->settings.baud_idx];
                t->line_start_tick=lt-tx_ms;
                t->line[t->line_len]='\0'; parse_line(t,t->line); t->line_len=0;
            }
            else if(c!='\r') { if(t->line_len<sizeof(t->line)-1) t->line[t->line_len++]=c; else t->line_len=0; }
        }
        t->nl_tail=t->nl_head; /* queue drained: forget any unmatched line stamps */
        tilestore_tick(&t->tiles);
        tz_update(t);
        trail_clock_sync(t);
        lastpos_tick(t,false);
        if(t->opt_state==2) optimizer_run(t);
        nav_tick(t);
        area_update(t);
        route_edit_poll(t);
        UiEvent ev;
        if(furi_message_queue_get(uiq,&ev,100)==FuriStatusOk) {
            if(ev.event.key==InputKeyBack && ev.event.type==InputTypeLong) {
                /* exit only from the home screens; ignored while recording */
                if(t->mode!=ModeRecording && !t->route_edit) running=false;
            } else if(t->settings_open && (ev.event.type==InputTypeShort ||
                                           ev.event.type==InputTypeLong ||
                                           ev.event.type==InputTypeRepeat)) {
                /* settings: Left/Right scroll, OK changes, Up nudges (time zone),
                   Down (or Back) returns. Holding Left/Right/Up repeats. */
                bool is_short=(ev.event.type==InputTypeShort);
                if(ev.event.key==InputKeyLeft) {
                    if(t->settings_page>0) t->settings_page--;
                } else if(ev.event.key==InputKeyRight) {
                    if(t->settings_page<SettingsPageCount-1) t->settings_page++;
                } else if(ev.event.key==InputKeyUp) {
                    settings_nudge(t);
                } else if(ev.event.key==InputKeyOk && is_short) {
                    settings_change(t,serial);
                } else if(ev.event.key==InputKeyOk && ev.event.type==InputTypeLong &&
                          t->settings_page==SettingsTrim) {
                    trim_set(t,0);
                } else if(ev.event.key==InputKeyOk && ev.event.type==InputTypeLong &&
                          t->settings_page==SettingsDestination) {
                    settings_delete_dest(t);
                } else if((ev.event.key==InputKeyDown || ev.event.key==InputKeyBack) && is_short) {
                    t->settings_open=false;
                    if(t->mode==ModeHome) t->home_page=HomePageMenu;
                }
            } else if(trail_on_route(t) &&
                      (ev.event.key==InputKeyUp || ev.event.key==InputKeyDown ||
                       ev.event.key==InputKeyOk) &&
                      (ev.event.type==InputTypeShort || ev.event.type==InputTypeLong ||
                       ev.event.type==InputTypeRepeat)) {
                route_page_input(t,&ev.event);
            } else if(trail_on_map(t) && t->map_free &&
                      (ev.event.type==InputTypeShort || ev.event.type==InputTypeLong ||
                       ev.event.type==InputTypeRepeat)) {
                /* free-move: arrows pan, OK or Back finishes */
                bool is_short=(ev.event.type==InputTypeShort);
                if((ev.event.key==InputKeyOk || ev.event.key==InputKeyBack) && is_short)
                    map_free_exit(t);
                else if(ev.event.key==InputKeyOk && ev.event.type==InputTypeLong)
                    map_pin(t,true);
                else
                    map_pan(t,ev.event.key);
            } else if(trail_on_map(t) && !t->map_free && t->settings.free_move &&
                      ev.event.type==InputTypeLong && ev.event.key==InputKeyOk) {
                map_pin(t,false);
            } else if(ev.event.type==InputTypeShort) {
                if(ev.event.key==InputKeyUp) {
                    if(trail_settings_allowed(t)) {
                        if(trail_on_time(t)) t->settings_page=SettingsTimeZone;
                        t->settings_open=true;
                    }
                } else if(ev.event.key==InputKeyOk) {
                    if(trail_on_map(t) && t->settings.free_move) map_free_enter(t);
                } else if(ev.event.key==InputKeyBack) {
                    if(t->mode==ModeHome) {
                        t->home_page=HomePageMenu;
                        begin_recording(t);
                    } else if(t->mode==ModeRecording) {
                        t->mode=ModeHome;
                        t->idle=false;
                        t->have_prev=false;
                        t->have_sample_prev=false;
                        t->home_page=HomePageMenu;
                        tilestore_flush_all(&t->tiles);
                        lastpos_tick(t,true);
                        strlcpy(t->status,"SAVED",sizeof(t->status));
                    } else if(t->mode==ModeRetrace) {
                        t->mode=ModeHome;
                        t->home_page=HomePageMenu;
                    }
                } else if(ev.event.key==InputKeyLeft) {
                    if(t->mode==ModeHome) {
                        if(t->home_page>HomePageRoute) {
                            t->home_page=(TrailHomePage)(t->home_page-1);
                            if(t->home_page==HomePageMap && t->route_count==0 && t->route_path[0]) {
                                load_latest_route(t);
                            }
                        }
                    } else if(t->mode==ModeRecording) {
                        if(t->active_tab>TrailTabRoute) t->active_tab=(TrailTab)(t->active_tab-1);
                    } else if(t->mode==ModeRetrace) {
                        t->mode=ModeHome;
                        t->home_page=HomePageMenu;
                    }
                } else if(ev.event.key==InputKeyRight) {
                    if(t->mode==ModeHome) {
                        if(t->home_page<HomePageTime) t->home_page=(TrailHomePage)(t->home_page+1);
                    } else if(t->mode==ModeRecording) {
                        if(t->active_tab<(TrailTab)(TrailTabCount-1)) t->active_tab=(TrailTab)(t->active_tab+1);
                    } else if(t->mode==ModeRetrace) {
                        t->mode=ModeHome;
                        t->home_page=HomePageMenu;
                    }
                }
            }
        }
        {
            ViewPortOrientation want=trail_orientation(t);
            if(view_port_get_orientation(vp)!=want) view_port_set_orientation(vp,want);
        }
        view_port_update(vp);
        if(t->opt_state==1) t->opt_state=2; /* the 'Working...' text is on screen: run it next round */
    }
    furi_hal_serial_async_rx_stop(serial); furi_hal_serial_deinit(serial); furi_hal_serial_control_release(serial);
    expansion_enable(expansion); furi_record_close(RECORD_EXPANSION);
    gui_remove_view_port(gui,vp); furi_record_close(RECORD_GUI); view_port_free(vp);
    furi_message_queue_free(uiq); furi_message_queue_free(t->rx_queue);
    lastpos_tick(t,true);
    tilestore_free(&t->view);
    tilestore_free(&t->tiles);
    furi_mutex_free(t->mutex); free(t);
    return 0;
}
