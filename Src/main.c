#include <main.h>

static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

volatile uint8_t flag = 0;

void EXTI15_10_IRQHandler() {
	__HAL_GPIO_EXTI_CLEAR_IT(TS_INT_PIN);
	flag = 1;
}

void drawMandelbrotAlternative(const uint8_t scale, const uint16_t iterations,
		const uint16_t centerX, const uint16_t centerY) {
	const uint16_t height = 272;
	const uint16_t width = 480;
	const float scaleFactor = 4.0 / width / scale;
	float c_re, c_im, z_re, z_re_temp, z_im_temp, z_im, n;
	const float halfIterations = iterations / 2;
	const float color_factor = 255 / iterations;
	uint8_t shade_red = 255;
	uint32_t col = 0xFFFF0000;
	char isInside = 1;
	uint16_t y, x;

	for (y = 0; y < height; y++) {
		c_im = (y - centerY) * scaleFactor;
		for (x = 0; x < width; x++) {
			c_re = (x - centerX) * scaleFactor;
			z_re = 0;
			z_re_temp = 0;
			z_im_temp = 0;
			z_im = 0;
			isInside = 1;

			for (n = 0; n < iterations; n++) {
				z_re_temp = z_re * z_re;
				z_im_temp = z_im * z_im;
				if (z_re_temp + z_im_temp > 4) {
					isInside = 0;
					break;
				}
				z_im = 2 * z_re * z_im + c_im;
				z_re = z_re_temp - z_im_temp + c_re;
			}
			if (isInside) {
				BSP_LCD_DrawPixel(x, y, 0xFF000000);
			} else if (n > halfIterations) {
				shade_red = n * color_factor;
				col = (shade_red << 8) + (shade_red << 16) + (255 << 24);
				BSP_LCD_DrawPixel(x, y, col);
			} else {
				shade_red = n * color_factor;
				col = (shade_red << 16) + (255 << 24);
				BSP_LCD_DrawPixel(x, y, col);
			}
		}
	}
}

int main(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	CPU_CACHE_Enable();

	HAL_Init();

	SystemClock_Config();

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(0);

	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	BSP_TS_ITConfig();

	TS_StateTypeDef ts;
	uint8_t globalScale = 1;
	uint16_t iterations = 1;
	uint16_t centerX = 240;
	uint16_t centerY = 136;

	while (1) {
		if (flag) {
			BSP_TS_GetState(&ts);
			if (!ts.touchDetected) {
				flag = 0;
				globalScale *= 2;
				centerX = BSP_LCD_GetXSize() - ts.touchX[0];
				centerY = BSP_LCD_GetYSize() - ts.touchY[0];
			}
		}
		drawMandelbrotAlternative(globalScale, iterations++, centerX, centerY);
		HAL_Delay(5);
	}
}

void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;

	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}

	ret = HAL_PWREx_EnableOverDrive();
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
	if (ret != HAL_OK) {
		while (1) {
			;
		}
	}
}

static void CPU_CACHE_Enable(void) {
	SCB_EnableICache();
	SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
