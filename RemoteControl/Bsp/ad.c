#include "main.h"
#include "adc.h"

/*DMA 目标缓冲区长度，与 ADC 规则组的 Rank 数量一一对应*/
#define AD_CHANNEL_COUNT		4

/*DMA 搬运目标缓冲区，volatile 告知编译器这块内存会被硬件随时改写*/
volatile uint16_t AD_Value[AD_CHANNEL_COUNT];

/**
  * 函    数：AD 初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：ADC1 已由 CubeMX 配成 4 通道扫描 + 连续转换 + DMA1_Channel1 循环搬运，
  *           这里只做上电校准，并把 DMA 启动起来；之后转换全程由硬件在后台完成，
  *           主循环再也不用为采样等待
  */
void AD_Init(void)
{
	/*校准必须在 ADC 关闭状态下进行，此时 DMA 还未启动*/
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	/*启动后 ADC 不间断扫描 4 个通道，DMA 循环覆盖缓冲区*/
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AD_Value, AD_CHANNEL_COUNT) != HAL_OK)
	{
		Error_Handler();		//起不来就不能继续跑：缓冲区恒为 0，换算后是满舵
	}
	
	/*Circular 模式下 DMA 每搬完 4 个值就产生一次传输完成中断，约 44kHz，
	  而本工程只读缓冲区、不需要任何回调。CubeMX 的 ForceEnableDMAVector 会在
	  MX_DMA_Init 里把 NVIC 重新使能，所以必须在这里运行期关掉中断源和 NVIC，
	  否则 CPU 每 23us 被打断一次，还占用比 1ms 节拍更高的优先级*/
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}

/**
  * 函    数：AD 读取四路摇杆的最新转换值
  * 参    数：LH / LV / RH / RV 输出四路原始值的指针，范围 0~4095
  * 返 回 值：无
  * 说    明：直接取 DMA 缓冲区，不启动转换也不阻塞，单次耗时只有几条指令。
  *           AD_Value[0..3] 对应 Rank1~Rank4，即 CH0/CH1/CH2/CH3。
  *           注意四路值来自连续转换的不同时刻（每路相差约 5.7us），
  *           对摇杆这种慢变量没有影响，但不能当同步采样用
  */
void AD_Read(uint16_t *LH, uint16_t *LV, uint16_t *RH, uint16_t *RV)
{
	*LH = AD_Value[0];
	*LV = AD_Value[1];
	*RH = AD_Value[2];
	*RV = AD_Value[3];
}
