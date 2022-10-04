
//#include "driver/gpio.h"
#include "driver/sdmmc_host.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"

#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdio.h>
#include <dirent.h>

#include <sys/unistd.h>
#include <sys/stat.h>

// ESP32-CAM Pins
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// SD Card Pins
#define SD_PIN_CLK      14
#define SD_PIN_CMD      15
#define SD_PIN_D0        2
#define SD_PIN_D1        4
#define SD_PIN_D2       12
#define SD_PIN_D3       13

// Input Pins/Misc
#define REC_QUEUE_SIZE  8
#define REC_QUALITY     12
#define REC_EN_PIN      16
#define REC_HI_WATER    3800000000
//#define REC_HI_WATER    1000000

#define SD_MOUNT        "/sd"
#define VIDEO_FNAME     "video_%d.mjpg"

static const char * TAG = "main";

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = REC_QUALITY, //0-63 lower number means higher quality
    .fb_count = REC_QUEUE_SIZE, //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_LATEST//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

typedef struct {
    int64_t timestamp_us;
    camera_fb_t * fb;
} cam_frame_t;

typedef struct {
    QueueHandle_t rtos_fb_queue;
    char * video_path;
    struct {
        uint32_t cam_ready : 1;
        uint32_t cam_recording : 1;
    };
} cam_sys_t;

void sd_open_next(FILE ** fjpg, int * fjpg_fd) {
    DIR * dir;
    struct dirent * entry;
    
    dir = opendir(SD_MOUNT);
    
    if (dir == NULL) {
        *fjpg = NULL;
        *fjpg_fd = -1;
        return;
    }
    
    int max_vidnum = -1;
    
    while ((entry = readdir(dir)) != NULL) {
        int vid_num = -1;
        //ESP_LOGV(TAG, "scan file %s", entry->d_name);
        if (sscanf(entry->d_name, VIDEO_FNAME, &vid_num) > 0
            && vid_num > max_vidnum) max_vidnum = vid_num;
    }
    
    max_vidnum++;
    
    char fjpg_path[256] = "\0";
    
    sprintf(fjpg_path, SD_MOUNT "/" VIDEO_FNAME, max_vidnum);
    
    ESP_LOGI(TAG, "opening %s for writing...", fjpg_path);
    
    *fjpg = fopen(fjpg_path, "w");
            
    if (*fjpg == NULL) {
        ESP_LOGE(TAG, "could not open %s for writing, panicing!", fjpg_path);
        esp_restart();
    }

    *fjpg_fd = fileno(*fjpg);
}

/*
SD task

recieves frames from a framebuffer queue and writes them to the SD card.
starts with a closed video file.

if the video file is null, it will read the latest video path and attempt to open the video file.

if a recieved framebuffer is null (end of stream), it will close the current video file.
otherwise, it will write the framebuffer contents to the file.

*/
void sd_write_task(void * arg) {
    cam_sys_t * cam_sys = (cam_sys_t *)arg;
    cam_frame_t frame;
    FILE * fjpg = NULL; 
    int fjpg_fd = -1;
    size_t fjpg_size = 0;
    
    ESP_LOGI(TAG, "sd write task ready!");
    
    while (xQueueReceive(cam_sys->rtos_fb_queue, (void *)&frame, portMAX_DELAY)) {
        
        if (fjpg == NULL) { // null file
            sd_open_next(&fjpg, &fjpg_fd);
            fjpg_size = 0;
        }
        
        if (frame.fb) { // valid frame
            fjpg_size += fwrite(frame.fb->buf, 1, frame.fb->len, fjpg);
            fsync(fjpg_fd);
            esp_camera_fb_return(frame.fb);
        } 
        
        if (!frame.fb || fjpg_size > REC_HI_WATER) { // if the frame is null or the high water mark has been reached
            fclose(fjpg);
            fjpg = NULL;
            fjpg_fd = -1;
        }
    }
}

int app_main() {
    cam_sys_t cam_sys;
        
    // mount sd
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = SD_MOUNT;
    
    ESP_LOGI(TAG, "initializing sdmmc card:");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();    
    slot_config.width = 4;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    ESP_LOGI(TAG, "mounting sd card to " SD_MOUNT);
    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card));
    ESP_LOGI(TAG, SD_MOUNT " successfully mounted");
    
    // create framebuffer queue
    cam_sys.rtos_fb_queue = xQueueCreate(REC_QUEUE_SIZE, sizeof(cam_frame_t));
    
    if (cam_sys.rtos_fb_queue == NULL) {
        ESP_LOGE(TAG, "failed to create framebuffer queue!");
    }
    
    // init camera
    if (CAM_PIN_PWDN != -1) { // disable powerdown
        gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level(CAM_PIN_PWDN, 0);
    }
    
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));
    
    ESP_LOGI(TAG, "camera ok");
    
    // start sd card task
    // esp camera task is pinned to core 0
    
    ESP_LOGI(TAG, "starting sd write task...");
    xTaskCreate(
        sd_write_task,
        "SDWrite",
        8192,
        &cam_sys,
        1,
        NULL
    );
    
    ESP_LOGI(TAG, "starting recording...");
    
    while (true) {
        cam_frame_t frame = {
            .timestamp_us = esp_timer_get_time(),
            .fb = esp_camera_fb_get()
        };
        
        if (!frame.fb) {
            ESP_LOGW(TAG, "camera framebuffer is null!");
            continue;
        }
        
        if (xQueueSendToBack(cam_sys.rtos_fb_queue, (void *)&frame, 10) != pdPASS) {
            ESP_LOGW(TAG, "dropped frame!");
        }
    }
        
    return 0;
}
