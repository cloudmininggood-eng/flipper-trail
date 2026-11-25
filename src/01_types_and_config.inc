/* flipper_trail module: types and config. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
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
