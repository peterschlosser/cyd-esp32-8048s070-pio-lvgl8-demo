# cyd-esp32-8048s070-pio-lvgl8-demo
A simplified PlatformIO project to dispel some of the mystery in setting up the LvGL 8 Widgets Demo for use with larger CYD displays.
## CYD ESP32-8048S070 + PlatformIO + LvGL 8 Widgets Demo
### Features
- Step-by-step instructions for PlatformIO + LvGL 8 on larger CYD displays using the ESP32-S3-WROOM-1 MCU Module
- Easily adapted for use with SquareLine Studio and EEZ Studio projects
- Implements
-- ESP32-S3 PSRAM Memory
-- DMA memory transfer
-- LvGL Widgets Demo
- Tested with
-- Sunton ESP32-8040S050
-- Sunton ESP32-8040S070
-- Guition ESP32-8040S050
-- Guition ESP32-8040S070
- Supports
-- LvGL 8.4.0
-- LovyanGFX 1.2.7

##Setup Procedure
1. Install Visual Studio Code IDE (VSCode)

2. Install PlatformIO VSCode Extension

3. Install the Espressif32 platform within PlatformIO

4. Copy the `boards/sunton_s3.json` profile into the espressif32 boards folder located (Win11) `C:/Users/[user]/.platformio/platforms/espressif32/boards/`

5. Create a new PlatformIO project using the `Sunton ESP32-S3` Board type and `Arduino` Framework

6. Save the VSCode workspace

7. Install the LvGL 8.4.0 Library within PlatformIO, or any newer version below 9.0.0.

8. Install the Lovyan GFX 1.2.7 (or newer) Library within PlatformIO

	All software components are now installed.

9. Modify the `platformio.ini` in the following ways:
	9.1. Set the serial port monitor speed to `115200`:
		[env:sunton_s3]
		platform = espressif32
		board = sunton_s3
		framework = arduino
		monitor_speed = 115200

		lib_deps =
			lovyan03/LovyanGFX@^1.2.7
			lvgl/lvgl@^8.4.0
	9.2. optional: enable GNU++17 C++ compiler, optimize output for size, and set ESP32 log level to INFO:
		[env:sunton_s3]
		platform = espressif32
		board = sunton_s3
		framework = arduino
		monitor_speed = 115200

		build_unflags = 
			-std=gnu++11
		build_flags = 
			-Os
			-std=gnu++17
			-DCORE_DEBUG_LEVEL=3

		lib_deps =
			lovyan03/LovyanGFX@^1.2.7
			lvgl/lvgl@^8.4.0

	9.3. enable storing the `lv_conf.h` file in the project `lib` folder, and protect it from being erased by a "Full Clean" operation:
		[env:sunton_s3]
		platform = espressif32
		board = sunton_s3
		framework = arduino
		monitor_speed = 115200

		build_unflags = 
			-std=gnu++11
		build_flags = 
			-Os
			-std=gnu++17
			-DCORE_DEBUG_LEVEL=3
			-DLV_CONF_INCLUDE_SIMPLE
			-I./lib

		lib_deps =
			lovyan03/LovyanGFX@^1.2.7
			lvgl/lvgl@^8.4.0

	9.4. save the changes to `platformio.ini`

10. Create the `lv_conf.h` file.  
	Copy the template file from LvGL library to project Lib folder.  
	Copy from: `.pio/libdeps/sunton_s3/lvgl/lv_conf_template.h` 
	and copy-rename to: `lib/lv_conf.h`

11. Modify the `lv_conf.h` file in the following ways:

	11.1 enable its content by setting the first `#if` to `1`
	```cpp
	#if 1 /*Set it to "1" to enable content*/```

	11.2 optional: increase the `LV_MEM_SIZE` to `1MB` as future projects will use up the buffer at its default size:
	```cpp
	#define LV_MEM_SIZE (1024U * 1024U)```

	11.3 change the type of memory from which buffers are allocated to PSRAM:
	```cpp
	#if LV_MEM_ADR == 0
		#undef LV_MEM_POOL_INCLUDE
		#undef LV_MEM_POOL_ALLOC
		#define LV_MEM_POOL_INCLUDE     "esp_heap_caps.h"
		#define LV_MEM_POOL_ALLOC(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
	#endif```
	11.4 enable `LV_TICK_CUSTOM` so we eliminate the need to call `lv_tick_inc()` from our main `loop()` and simplify our code:
	```cpp
	#define LV_TICK_CUSTOM 1```

	11.5 optional: enable the logging of messages in case something goes wrong:
	```cpp
	#define LV_USE_LOG 1```

	11.6 optional: enable the performance monitor to show us CPU usage and FPS count:
	```cpp
	#define LV_USE_PERF_MONITOR 1```

	11.7 optional: enable memory monitor to show is `LV_MEM_SIZE` consumption of this and future projects:
	```cpp
	#define LV_USE_MEM_MONITOR 1```

	11.8 enable the LvGL widgets demo as our example display content:
	```cpp
	#define LV_USE_DEMO_WIDGETS 1```

	11.9 save changes to `lv_conf.h`

12. as of this writing, when using LvGL 8.4.0, the `demos` folder containing the widgets demo must be copied in order for it to be compiled and correctly linked to the firmware binary.  this step is unnecessary if our project does not use the widgets demo.  
	Copy the whole `demos` folder from: `.pio/libdeps/sunton_s3/lvgl/demos` into the `src` folder: `.pio/libdeps/sunton_s3/lvgl/src` such that when complete, there is a copy of the `demos` folder at: `.pio/libdeps/sunton_s3/lvgl/src/demos` 
	Also, be aware platformio may update the `libdeps` (library dependencies) from time to time, and when it does, it may remove the copied `demos` folder.
	When building, if we see the error
	>undefined reference to 'lv_demo_widgets'

	we may need to copy the `demos` folder anew.

13. modify the `main.cpp` file in the following ways:

	13.1 clean up the default `main.cpp` content and prepare it for our changes to the `setup()` and `loop()` functions.  use ALT-Shift-F (CMD-Shift-F) to periodically reformat the content.
	```cpp
	#include <Arduino.h>

	void setup()
	{
		// put your setup code here, to run once:
	}

	void loop()
	{
		// put your main code here, to run repeatedly:
	}```

	13.2 modify `setup()` to prepare the `Serial` interface to display log messages:
	```cpp
	static const char *TAG = "main"; // logging

	void setup()
	{
		Serial.begin(115200);
		while (!Serial)
		{
			delay(100);
		}
		ESP_LOGI(TAG, "started.");
	}```

	13.3 **test Serial.** compile and upload the firmware to our device.  open the PlatformIO Serial Monitor and observe the "started" message.  this is a good place to resolve any USB connection issues with our device, and any compiler or build errors.  on success, our serial monitor may look something like:
		--- Terminal on COM7 | 115200 8-N-1
		--- Available filters and text transformations: colorize, debug, default, direct, esp32_exception_decoder, hexlify, log2file, nocontrol, printable, send_on_enter, time
		--- More details at https://bit.ly/pio-monitor-filters
		--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
		ESP-ROM:esp32s3-20210327
		Build:Mar 27 2021
		rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
		SPIWP:0xee
		mode:DIO, clock div:1
		load:0x3fce3808,len:0x4bc
		load:0x403c9700,len:0xbd8
		load:0x403cc700,len:0x2a0c
		entry 0x403c98d0
		[   105][I][esp32-hal-psram.c:96] psramInit(): PSRAM enabled
		[   126][I][main.cpp:12] setup(): [main] started.

14. setup and test the LovyanGFX driver for our device.  modify the main.cpp file in the following ways:

	14.1 to the top of main.cpp, add the #includes of the lovyan driver:
	```cpp
	#include <LovyanGFX.hpp>
	#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
	#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
	#include <driver/i2c.h>```

	14.2 below the TAG variable, add the LGFX class declaration specific to the device:
	```cpp
	class LGFX : public lgfx::LGFX_Device
	{
	public:
		lgfx::Bus_RGB _bus_instance;
		lgfx::Panel_RGB _panel_instance;
		lgfx::Light_PWM _light_instance;
		lgfx::Touch_GT911 _touch_instance;

		LGFX(void)
		{
			{
				auto cfg = _panel_instance.config();

				cfg.memory_width = 800;
				cfg.memory_height = 480;
				cfg.panel_width = 800;
				cfg.panel_height = 480;

				cfg.offset_x = 0;
				cfg.offset_y = 0;

				_panel_instance.config(cfg);
			}

			{
				auto cfg = _panel_instance.config_detail();

				cfg.use_psram = 1;

				_panel_instance.config_detail(cfg);
			}

			{
				auto cfg = _bus_instance.config();
				cfg.panel = &_panel_instance;
				cfg.pin_d0 = GPIO_NUM_15;  // B0
				cfg.pin_d1 = GPIO_NUM_7;   // B1
				cfg.pin_d2 = GPIO_NUM_6;   // B2
				cfg.pin_d3 = GPIO_NUM_5;   // B3
				cfg.pin_d4 = GPIO_NUM_4;   // B4
				cfg.pin_d5 = GPIO_NUM_9;   // G0
				cfg.pin_d6 = GPIO_NUM_46;  // G1
				cfg.pin_d7 = GPIO_NUM_3;   // G2
				cfg.pin_d8 = GPIO_NUM_8;   // G3
				cfg.pin_d9 = GPIO_NUM_16;  // G4
				cfg.pin_d10 = GPIO_NUM_1;  // G5
				cfg.pin_d11 = GPIO_NUM_14; // R0
				cfg.pin_d12 = GPIO_NUM_21; // R1
				cfg.pin_d13 = GPIO_NUM_47; // R2
				cfg.pin_d14 = GPIO_NUM_48; // R3
				cfg.pin_d15 = GPIO_NUM_45; // R4

				cfg.pin_henable = GPIO_NUM_41;
				cfg.pin_vsync = GPIO_NUM_40;
				cfg.pin_hsync = GPIO_NUM_39;
				cfg.pin_pclk = GPIO_NUM_42;
				cfg.freq_write = 12000000; // reduce jitter

				cfg.hsync_polarity = 0;
				cfg.hsync_front_porch = 80;
				cfg.hsync_pulse_width = 4;
				cfg.hsync_back_porch = 16;
				cfg.vsync_polarity = 0;
				cfg.vsync_front_porch = 22;
				cfg.vsync_pulse_width = 4;
				cfg.vsync_back_porch = 4;
				cfg.pclk_idle_high = 1;
				_bus_instance.config(cfg);
			}
			_panel_instance.setBus(&_bus_instance);

			{
				auto cfg = _light_instance.config();
				cfg.pin_bl = GPIO_NUM_2;
				_light_instance.config(cfg);
			}
			_panel_instance.light(&_light_instance);

			{
				auto cfg = _touch_instance.config();
				cfg.x_min = 0;
				cfg.x_max = 800;
				cfg.y_min = 0;
				cfg.y_max = 480;
				cfg.pin_int = GPIO_NUM_NC;
				cfg.bus_shared = true;
				cfg.offset_rotation = 0;
				cfg.i2c_port = I2C_NUM_1;
				cfg.pin_sda = GPIO_NUM_19;
				cfg.pin_scl = GPIO_NUM_20;
				cfg.pin_rst = GPIO_NUM_38;
				cfg.freq = 400000;
				cfg.i2c_addr = 0x14; // 0x5D , 0x14
				_touch_instance.config(cfg);
				_panel_instance.setTouch(&_touch_instance);
			}

			setPanel(&_panel_instance);
		}
	};```

	14.3 below the LGFX class, use it to declare the gfx variable:
	```cpp
	LGFX gfx;	```

	14.4 below the "started." log entry within `setup()`, add the following lines to setup GFX and use of the display:
	```cpp
	gfx.begin();
	gfx.initDMA();
	gfx.setBrightness(128);           // adjust as desired (1-255)
	gfx.setCursor(0, 0);              // optional
	gfx.setTextColor(0xFFFF, 0x0000); // optional
	gfx.setTextSize(1);               // optional
	gfx.setTextWrap(false);           // optional
	ESP_LOGI(TAG, "display started.");```

	14.5 **test display.** below the "display started." log entry within `setup()`, add the following two lines of test code, compile and upload the firmware to our device.  we should observe three (3) changes, the log entry "display started" in the serial monitor, a red rectangle in the middle of the display, and the text "hello, world." in the upper left corner.  once tested, remove the two test lines of code from the bottom of `setup()`.
	```cpp
	gfx.fillRect(200, 120, 400, 240, 0xF800);
	gfx.println("hello, world.");```

	14.6 **test touch.** within the `loop()` function, add the following lines of test code, compile and upload the firmware to our device.  as we touch the display, we should observe log entries in the serial monitor telling us the location of the touches.  once tested, remove these test lines of code from the `loop()` function.
	```cpp
	uint16_t touchX, touchY;
	bool touched = gfx.getTouch(&touchX, &touchY);
	if (touched)
	{
		ESP_LOGI(TAG, "display touched at: %d, %d.", touchX, touchY);
	}```
15. setup and test the LvGL library using the widgets demo as our display content.

	15.1 to the top of `main.cpp`, add the #includes of the LvGL library and widgets demo:
	```cpp
	#include <lvgl.h>
	#include <demos/lv_demos.h>```

	15.2 below the `gfx` variable declaration above `setup()`, add the following variables to define our display, touch and buffer configurations.
	```cpp
	#define _LV_DISP_DRAW_BUF_SIZE (gfx.width() * gfx.height() * sizeof(lv_color_t) / 8)
	lv_disp_drv_t disp_drv;
	lv_indev_drv_t indev_drv;
	lv_disp_draw_buf_t disp_draw_buf;
	lv_color_t *disp_buf[2];```

	15.3 declare the "buffer flush" callback used by LvGL to write content to the display.  above `setup()` add the following `my_disp_flush()` function:
	```cpp
	void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
	{
		if (gfx.getStartCount() == 0)
		{
			gfx.startWrite();
		}
		gfx.pushImageDMA(area->x1,
											area->y1,
											area->x2 - area->x1 + 1,
											area->y2 - area->y1 + 1,
											(lgfx::rgb565_t *)&color_p->full);
		gfx.endWrite();
		lv_disp_flush_ready(disp);
	}```

	15.4 declare the "touch read" callback used by LvGL to know when and where the display was touched.  above `setup()` add the following `my_touchpad_read()` function:
	```cpp
	void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
	{
		uint16_t touchX, touchY;
		const uint32_t touchDebounce = 10; // ms
		static uint32_t lastTouchTime = 0;
		if (millis() - lastTouchTime < touchDebounce)
		{
			return;
		}

		bool touched = gfx.getTouch(&touchX, &touchY);
		data->state = LV_INDEV_STATE_RELEASED;

		if (touched)
		{
			lastTouchTime = millis();
			data->state = LV_INDEV_STATE_PRESSED;
			data->point.x = touchX;
			data->point.y = touchY;
		}
	}```

	15.5 allocate and initialize the draw buffers used by LvGL.  add the following lines to the bottom of `setup()`:
	```cpp
	lv_init();
	disp_buf[0] = (lv_color_t *)heap_caps_aligned_alloc(64, _LV_DISP_DRAW_BUF_SIZE, /* MALLOC_CAP_DMA | */ MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	disp_buf[1] = (lv_color_t *)heap_caps_aligned_alloc(64, _LV_DISP_DRAW_BUF_SIZE, /* MALLOC_CAP_DMA | */ MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (disp_buf[0] == nullptr || disp_buf[1] == nullptr)
	{
		ESP_LOGI(TAG, "buffer allocation failure, aborting.");
		while (1)
			yield();
	}
	lv_disp_draw_buf_init(&disp_draw_buf, disp_buf[0], disp_buf[1], _LV_DISP_DRAW_BUF_SIZE / /* pixel size */ sizeof(lv_color_t));```

	15.6 initialize and register the display driver with LvGL.  add the following lines to the bottom of `setup()`:
```cpp
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = gfx.width();
	disp_drv.ver_res = gfx.height();
	disp_drv.flush_cb = my_disp_flush;
	disp_drv.draw_buf = &disp_draw_buf;
	disp_drv.full_refresh = 0;
	lv_disp_drv_register(&disp_drv);```

	15.7 initialize and register the input driver with LvGL.  add the following lines to the bottom of `setup()`:
	```cpp
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_touchpad_read;
	lv_indev_drv_register(&indev_drv);```

	15.8 prepare the main `loop()` to drive LvGL and do its thing.  add the following lines to the `loop()` function:
	```cpp
	lv_timer_handler();

	delay(1);```

	15.9 finally, prepare the widgets demo as the screen content.  add the follwing lines to the botton of `setup()`:
	```cpp
	lv_demo_widgets();
	ESP_LOGI(TAG, "gui started.");```

	15.10 **test LvGL.** compile and upload the firmware to our device.  we should observe the "gui started" message in the serial monitor and a rich demo of widgets on the display.


