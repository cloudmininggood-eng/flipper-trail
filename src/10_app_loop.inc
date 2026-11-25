/* flipper_trail module: app loop. Included by flipper_trail.c (one translation unit),
   so order matters: see the include list there. */
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
