/*
 * Simple Anchor / Tag configurator for a Real-Time Location System.
 * Author: Jonathan Pereira
 */

 #include "dwm.h"
 #include <stdio.h>
 
 /* Thread priority */
 #ifndef THREAD_APP_PRIO
 #define THREAD_APP_PRIO	20
 #endif /* THREAD_APP_PRIO */
 
 /* Thread stack size */
 #ifndef THREAD_APP_STACK_SIZE
 #define THREAD_APP_STACK_SIZE	(3 * 1024)
 #endif /* THREAD_APP_STACK_SIZE */
 
 #define APP_ERR_CHECK(err_code)	\
 do {							\
   if ((err_code) != DWM_OK)	\
     printf("err: line(%u) code(%u)", __LINE__, (err_code));\
 } while (0)						\
 
 #define MSG_INIT	\
   "\n\n"	\
   "App   :  dwm-simple\n"	\
   "Built :  " __DATE__ " " __TIME__ "\n"	\
   "\n"
 //kalman filter
 typedef struct{
   float q;
   float r;
   float p;
   float x;
 } KalmanFilter;
 static KalmanFilter ANCHOR[4];
 
 void kalman_init(KalmanFilter* kf, float q, float r, float initial_value){
   kf->q = q;
   kf->r = r;
   kf->p = 1.0f;
   kf->x = initial_value;
  }
 
 float kalman_update(KalmanFilter* kf, float measurement) {
     // Prediction
     kf->p = kf->p + kf->q;
     
     // Update
     float k = kf->p / (kf->p + kf->r);
     kf->x = kf->x + k * (measurement - kf->x);
     kf->p = (1 - k) * kf->p;
     
     return kf->x;
 }
 
 // Th?m h?m c?p nh?t R d?ng d?a tr?n QF
 float adaptive_kalman_update(KalmanFilter* kf, float measurement, uint8_t qf) {
     // V� qf lu�n l� 100, ta b? sung ki?m tra hi?u s? b?t thu?ng
     float diff = fabs(measurement - kf->x);
     float threshold = 50.0; // Ngu?ng cho hi?u s?, di?u ch?nh theo y�u c?u th?c t?
     if(diff > threshold) {
          // N?u ch�nh l?ch qu� l?n, coi d�y l� ngo?i lai -> tang R d? gi?m t�c d?ng c?a gi� tr? do hi?n t?i
          kf->r = 70.0;
     } else {
          // Gi� tr? R m?c d?nh, c� th? di?u ch?nh cho ph� h?p
          kf->r = 15.0;
     }
     
     // D? do�n
     kf->p = kf->p + kf->q;
     
     // C?p nh?t
     float k = kf->p / (kf->p + kf->r);
     kf->x = kf->x + k * (measurement - kf->x);
     kf->p = (1 - k) * kf->p;
     
     return kf->x;
 }
 /**
  * Event callback
  *
  * @param[in] p_evt  Pointer to event structure
  */
 void on_dwm_evt(dwm_evt_t *p_evt)
 {
   int len;    
   int i;
 
   switch (p_evt->header.id) {
   /* New location data */
   case DWM_EVT_LOC_READY:
     printf("\nT:%lu ", dwm_systime_us_get());
     if (p_evt->loc.pos_available) {
       printf("POS:[%ld,%ld,%ld,%u] ", p_evt->loc.pos.x,
           p_evt->loc.pos.y, p_evt->loc.pos.z,
           p_evt->loc.pos.qf);
     } else {
       printf("POS:N/A ");
     }
 
     for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
       printf("DIST%d:", i);
 
       printf("0x%04X", (unsigned int)(p_evt->loc.anchors.dist.addr[i] & 0xffff));
       if (i < p_evt->loc.anchors.an_pos.cnt) {
         printf("[%ld,%ld,%ld]",
             p_evt->loc.anchors.an_pos.pos[i].x,
             p_evt->loc.anchors.an_pos.pos[i].y,
             p_evt->loc.anchors.an_pos.pos[i].z);
       }
 
       printf("=[%lu,%u] ", p_evt->loc.anchors.dist.dist[i],
           p_evt->loc.anchors.dist.qf[i]);
     }
     printf("\n");
     break;
 
   case DWM_EVT_USR_DATA_READY:
     len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
     if (len <= 0)
       break;
 
     printf("iot received, len=%d:", len);
     for (i = 0; i < len; ++i) {
       printf(" %02X", p_evt->usr_data[i]);
     }
     break;
 
   case DWM_EVT_USR_DATA_SENT:
     printf("iot sent\n");
     break;
 
   case DWM_EVT_BH_INITIALIZED_CHANGED:
     printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
     break;
 
   case DWM_EVT_UWBMAC_JOINED_CHANGED:
     printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
     break;
 
   default:
     break;
   }
 }
 
 enum NodeType { Anchor = 0, Tag = 1 };
 
 void app_thread_entry(uint32_t data){
 
   /* Set the node type to either Anchor (0) or Tag (1) */
   enum NodeType node_type = Tag;
   int rv; //WTF is "rv"?
 
   /* Check and Verify the Firmware version of the module */
   dwm_ver_t ver;
   rv = dwm_ver_get(&ver);
   if (rv == DWM_OK){
     printf("FW Major: %d\n", ver.fw.maj);
     printf("FW Minor: %d\n", ver.fw.min);
     printf("FW Patch: %d\n", ver.fw.patch);
 
     printf("FW Res: %d\n", ver.fw.res);
     printf("FW Var: %d\n", ver.fw.var);
     printf("HW Version: %08x\n", ver.hw);
     printf("CFG Version: %08x\n", ver.cfg);
   }
 
 
   /* 
    * Configure the node as a Tag.
    * Set the Measurement mode to TWR.
    * Enable the Normal Update mode.
    * Enable Internal Location Engine.
    * Set the UWB Mode to Active.
    */
     // Th?m ph?n kh?i t?o b? l?c
		 for (int i = 0; i < 4; i++
			{
					kalman_init(&ANCHOR[i], 0.005, 15.0, 500.0); 
			}
     //cau hinh
     dwm_cfg_tag_t set_t_cfg;
     set_t_cfg.stnry_en = 0;
     set_t_cfg.meas_mode = DWM_MEAS_MODE_TWR;
     set_t_cfg.low_power_en = 0;
     set_t_cfg.loc_engine_en = 1;
     set_t_cfg.common.enc_en = 0;
     set_t_cfg.common.led_en = 1;
     set_t_cfg.common.ble_en = 1;
     set_t_cfg.common.fw_update_en = 0;
     set_t_cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
 
     dwm_cfg_tag_set(&set_t_cfg);
     if(rv == DWM_OK){
       printf("Tag Set Stationary Detection: %s\n", set_t_cfg.stnry_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set Measurement Mode: %s\n", set_t_cfg.meas_mode == 0 ? "TWR" : "Reserved");
       printf("Tag Set Low Power Mode: %s\n", set_t_cfg.low_power_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set Internal Location Engine: %s\n", set_t_cfg.loc_engine_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set Encryption: %s\n", set_t_cfg.common.enc_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set LEDs: %s\n", set_t_cfg.common.led_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set BLE: %s\n", set_t_cfg.common.ble_en == 0 ? "Disabled" : "Enabled");
       printf("Tag Set UWB Mode: %s\n", set_t_cfg.common.uwb_mode == 0 ? "Off" : set_t_cfg.common.uwb_mode == 1 ? "Passive" : "Active");
       printf("Tag Set FW Update: %s\n", set_t_cfg.common.fw_update_en == 0 ? "Disabled" : "Enabled");
     }
     else{
       printf("RV: %d\n", rv);
     }
 
     /* Verify if the node is configured as Tag correctly */
     dwm_cfg_t get_t_cfg;
     dwm_cfg_get(&get_t_cfg);
     printf("Tag Get mode %u \n", get_t_cfg.mode);
     printf("Tag Get initiator %u \n", get_t_cfg.initiator);
     printf("Tag Get bridge %u \n", get_t_cfg.bridge);
     printf("Tag Get motion detection enabled %u \n", get_t_cfg.stnry_en);
     printf("Tag Get measurement mode %u \n", get_t_cfg.meas_mode);
     printf("Tag Get low power enabled %u \n", get_t_cfg.low_power_en);
     printf("Tag Get internal location engine enabled %u \n", get_t_cfg.loc_engine_en);
     printf("Tag Get encryption enabled %u \n", get_t_cfg.common.enc_en);
     printf("Tag Get LED enabled %u \n", get_t_cfg.common.led_en);
     printf("Tag Get BLE enabled %u \n", get_t_cfg.common.ble_en);
     printf("Tag Get firmware update enabled %u \n", get_t_cfg.common.fw_update_en);
     printf("Tag Get UWB mode %u \n", get_t_cfg.common.uwb_mode); 
   /* Set the PAN ID of all the nodes in the network to 0x0001 */ 
   uint16_t panid = 0x0001;
   rv = dwm_panid_set(panid);
   if (rv == DWM_OK){
     printf("PAN ID: %x\n" , panid);
   }
   else{
     printf("%d\n", rv);
   }
 
   /* Set the default position of the node */
   dwm_pos_t pos;
   pos.qf = 100;
   pos.x = 1700; 
   pos.y = 0;
   pos.z = 0; 
   rv = dwm_pos_set(&pos);
   if(rv == DWM_OK){
     printf("Set Position Quality Factor: %d\n", pos.qf);
     printf("Set Position coordinates X: %d\n", pos.x);
     printf("Set Position coordinates Y: %d\n", pos.y);
     printf("Set Position coordinates Z: %d\n", pos.z);
   }
   else{
     printf("%d\n", rv);
   }
 
   /* Verify if the node position has been set correctly */
   dwm_pos_t gpos;
   rv = dwm_pos_get(&gpos);
   if (rv == DWM_OK){
     printf("Get Anchor Position [%d, %d, %d]\n", gpos.x, gpos.y, gpos.z);
   }
   else{
     printf("%d\n", rv);
   }
 
   printf("Configurations Completed\n");
 
 
   while(1){
     /* Read last measured distances to anchors (tag is currently ranging to) and the associated position */
       dwm_loc_data_t loc;
       int rv, i;
       rv = dwm_loc_get(&loc);
       if (rv == DWM_OK) {
         if(loc.pos_available)
         {
            //printf("number of anchor positions: %d   ->  ", loc.anchors.an_pos.cnt);
            //printf("[%d,%d,%d,%u] ", loc.pos.x, loc.pos.y, loc.pos.z, loc.pos.qf);
            //printf(" [%d] ", loc.anchors.dist.cnt);
         }
         for (i = 0; i < loc.anchors.dist.cnt; ++i) {
         if (i < loc.anchors.an_pos.cnt) {
 
             //printf("Raw data: 0x%04X: %lu ", 
             //loc.anchors.dist.addr[i],
             //loc.anchors.dist.dist[i]);
             //loc.anchors.dist.qf[i]);
           }
           // ?p d?ng Kalman Filter
                     float filtered_dist = 0;
                     filtered_dist = kalman_update(&ANCHOR[i], loc.anchors.dist.dist[i]);
            printf("0x%04x: ", loc.anchors.dist.addr[i]);
            printf("=%0.f | ", filtered_dist);
         }
         printf("\n");
       } 
       else {
         printf("err code: %d\n", rv);
       }
   }
 }
 
 
 /**
  * Application entry point. Initialize application thread.
  *
  * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
  * USER THREADS CAN BE DONE IN THIS FUNCTION
  */
 void dwm_user_start(void)
 {
         
   uint8_t hndl;
   int rv;
   dwm_shell_compile();
   //Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
   dwm_ble_compile();
   dwm_le_compile();
   dwm_serial_spi_compile();
 
   /* Create thread */
   rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
       "app", THREAD_APP_STACK_SIZE, &hndl);
   APP_ERR_CHECK(rv);
 
   /* Start the thread */
   dwm_thread_resume(hndl);
 
 }
 