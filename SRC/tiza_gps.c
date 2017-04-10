#define GPS_STRUCT_GLOBAL

#include "tiza_gps.h"
#include "tiza_selfdef.h"

#include "tiza_include.h"

//#define GPS_DEBUG

#define GPS_MAX_BUF_LEN 				1024

typedef struct
{
	uint8 rx_done_flag;
	uint8 rx_buf[GPS_MAX_BUF_LEN];
	uint16 rx_counter;
	uint16 rx_timeout_decounter;
}GPS_BUF_STRUCT;


#define STATE_A_GPS					(0x01 << 24)
#define STATE_N_LAT					(0x01 << 25)
#define STATE_E_LONG				(0x01 << 26)


#define LAT_TYPE 	0
#define LONG_TYPE 	1
#define SPEED_TYPE 	2
#define DIR_TYPE 	3
#define AMP_TYPE 	4

#define RMC_TYPE 	1
#define GGA_TYPE 	2


#define ON_GPS_PWR()		(GPIO_SetBits(GPIOD, GPIO_Pin_9))
#define OFF_GPS_PWR()		(GPIO_ResetBits(GPIOD, GPIO_Pin_9))

#define GET_GPS_ANT_STATUS()		(GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_6))


static GPS_BUF_STRUCT g_gps_struct;
static uint8 gps_rx_data[GPS_MAX_BUF_LEN];

static NSS_INFO *g_nss_info = NULL;
static uint16 g_gps_recv_len = 0;
static bool g_gps_recv_ok = FALSE;

static bool skip_broken_result = FALSE;
static bool gps_ant_cut = FALSE;		// 天线是否被剪

// local function
static uint8 GpsFrameCheck(uint8 gps_data[],uint8 len);
static void GpsGetGprmcGpggaInfo(uint8 gps_data[],uint8 len,uint8 type);
static uint8 GpsCharFloatDataToHex(uint8 gps_data[],uint8 len,uint8 type,uint32 *r_val);
static uint8 GpsGetDate(uint8 utc_date[],uint8 hex_date[]);
static uint8 GpsGetTime(uint8 utc_time[],uint8 hex_time[]);

static uint8 GpsTryRxDone(void);


// BCD 转换
static void ToBcdLng(uint8 *bcd, uint8 *raw_data, uint8 raw_len)
{
	RamZero(bcd, 5);

	if(raw_len != 11)
	{
		return;
	}

	// 度
	*bcd |= (raw_data[0] - 0x30) & 0x0F;
	bcd++;

	*bcd |= ((raw_data[1] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[2] - 0x30) & 0x0F;
	bcd++;

	// 分
	*bcd |= ((raw_data[3] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[4] - 0x30) & 0x0F;
	bcd++;

	*bcd |= ((raw_data[6] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[7] - 0x30) & 0x0F;
	bcd++;

	*bcd |= ((raw_data[8] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[9] - 0x30) & 0x0F;
}

static void ToBcdLat(uint8 *bcd, uint8 *raw_data, uint8 raw_len)
{
	RamZero(bcd, 4);

	if(raw_len != 10)
	{
		return;
	}

	// 度
	*bcd |= ((raw_data[0] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[1] - 0x30) & 0x0F;
	bcd++;

	// 分
	*bcd |= ((raw_data[2] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[3] - 0x30) & 0x0F;
	bcd++;

	*bcd |= ((raw_data[5] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[6] - 0x30) & 0x0F;
	bcd++;

	*bcd |= ((raw_data[7] - 0x30) & 0x0F) << 4;
	*bcd |= (raw_data[8] - 0x30) & 0x0F;
}

static void ToBcdAltitude(uint8 *bcd, uint8 *raw_data, uint8 raw_len)
{
	uint8 i, int_num = 0, point_num = 0, point_index = 0;
	
	RamZero(bcd, 4);
	
	for(i=0; i < raw_len; i++)
	{
		if(raw_data[i] != '.')
		{
			int_num++;
		}
		else
		{
			point_num = raw_len - int_num -1;
			point_index = i;
			break;
		}
	}

	if(point_index == 0)
		return;

	if(raw_data[0] == '-')
	{
		*bcd = 0x2d;
		int_num -= 1;
	}
	else
	{
		*bcd = 0;
	}

	// 整数
	if(int_num == 1)
	{
		*(bcd+2) |= (raw_data[point_index-1] - 0x30) & 0x0F;
	}
	else if(int_num == 2)
	{
		*(bcd+2) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+2) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;
	}
	else if(int_num == 3)
	{
		*(bcd+2) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+2) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;

		*(bcd+1) |= (raw_data[point_index-3] - 0x30) & 0x0F;
	}
	else if(int_num == 4)
	{
		*(bcd+2) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+2) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;

		*(bcd+1) |= (raw_data[point_index-3] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-4] - 0x30) & 0x0F) << 4;
	}

	// 小数
	if(point_num > 0)
	{
		*(bcd+3) = raw_data[point_index+1] - 0x30;
	}
}

static void ToBcdSpeed(uint8 *bcd, uint8 *raw_data, uint8 raw_len)
{
	uint8 i, int_num = 0, point_num = 0, point_index = 0;
	
	RamZero(bcd, 3);

	for(i=0; i < raw_len; i++)
	{
		if(raw_data[i] != '.')
		{
			int_num++;
		}
		else
		{
			point_num = raw_len - int_num -1;
			point_index = i;
			break;
		}
	}

	if(point_index == 0)
		return;

	// 整数
	if(int_num == 1)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
	}
	else if(int_num == 2)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;
	}
	else if(int_num == 3)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;

		*(bcd) |= (raw_data[point_index-3] - 0x30) & 0x0F;
	}
	else if(int_num == 4)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;

		*(bcd) |= (raw_data[point_index-3] - 0x30) & 0x0F;
		*(bcd) |= ((raw_data[point_index-4] - 0x30) & 0x0F) << 4;
	}

	// 小数
	if(point_num > 0)
	{
		*(bcd+2) = raw_data[point_index+1] - 0x30;
	}
}

static void ToBcdDirection(uint8 *bcd, uint8 *raw_data, uint8 raw_len)
{
	uint8 i, int_num = 0, point_num = 0, point_index = 0;
	
	RamZero(bcd, 3);

	for(i=0; i < raw_len; i++)
	{
		if(raw_data[i] != '.')
		{
			int_num++;
		}
		else
		{
			point_num = raw_len - int_num -1;
			point_index = i;
			break;
		}
	}

	if(point_index == 0)
		return;
	
	// 整数
	if(int_num == 1)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
	}
	else if(int_num == 2)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;
	}
	else if(int_num == 3)
	{
		*(bcd+1) |= (raw_data[point_index-1] - 0x30) & 0x0F;
		*(bcd+1) |= ((raw_data[point_index-2] - 0x30) & 0x0F) << 4;

		*(bcd) |= (raw_data[point_index-3] - 0x30) & 0x0F;
	}

	// 小数
	if(point_num > 0)
	{
		*(bcd+2) = raw_data[point_index+1] - 0x30;
	}
}


///1节=1海里/小时=1.852千米/小时

static uint8 GpsGetTime(uint8 utc_time[],uint8 hex_time[])
{
	uint8 i,res,u8_val;
	
	res = IsValidNum(utc_time,6);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<3;i++)
	{
		u8_val = (utc_time[i*2] - '0')*10;
		u8_val += utc_time[i*2+1] - '0';
		hex_time[i] = u8_val;
	}
	
	if((hex_time[0] >= 24)||(hex_time[1] >= 60)||(hex_time[2] >= 60))
	{
		res = FALSE;
	}
	else
	{
		res = TRUE;
	}
RETURN_LAB:
	return res;
}

static uint8 GpsGetDate(uint8 utc_date[],uint8 hex_date[])
{
	uint8 i,u8_val,res;
	
	res = IsValidNum(utc_date,6);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<3;i++)
	{
		u8_val = (utc_date[i*2] - '0')*10;
		u8_val += utc_date[i*2+1] - '0';
		hex_date[2-i] = u8_val;
	}
	
	if((hex_date[0] < 14 )||
	   (hex_date[0] >= 99)||
	   (hex_date[1] > 12 )||
	   (hex_date[1] < 1 )||
	   (hex_date[2] > 31)||
	   (hex_date[2] < 1))///年，月，日
	{
		res = FALSE;
	}
	else
	{
		res = TRUE;
	}
RETURN_LAB:
	return res;
}

static uint8 GpsCharFloatDataToHex(uint8 gps_data[],uint8 len,uint8 type,uint32 *r_val)///GPS字符浮点数转十六进制
{
	uint8 i,j,k,m,res;
	uint32 u32_val = 0,u32_val_div = 0;
	
	res = IsValidCharFloatNum(gps_data,len);
	if(!res)
	{
		goto RETURN_LAB;
	}
	
	for(i=0;i<len;i++)
	{
		if(gps_data[i] != '.')
		{
			u32_val *= 10;
			u32_val += gps_data[i] - 0x30;
			
		}
		else
		{
			j = len - i - 1;///小数点后的位数
			break;
		}
	}
	
	i++;
	if((type == LAT_TYPE)||(type == LONG_TYPE))///经纬度取小数点后5位
	{
		k = 0;
		if(j > 5)
		{
			j = 5;
		}
		else
		{
			if(j < 5)
			{
				k = 5 - j;///不足5位，补0个数
			}
		}
	}
	
	if((type == LAT_TYPE)||(type == LONG_TYPE))
	{
		u32_val_div = u32_val % 100;
		u32_val /= 100;
		u32_val *= 100;
		for(m=0;m<j;m++)///小数
		{
			u32_val *= 10;
			u32_val_div *= 10;
			u32_val_div += gps_data[i++] - 0x30;
		}
		
		for(m=0;m<k;m++)
		{
			u32_val *= 10;
			u32_val_div *= 10;
		}
		u32_val_div += 5;///l四舍五入
		
		u32_val_div = u32_val_div * 5;
		u32_val_div /= 3;
	}
	else
	{
		for(m=0;m<j;m++)///小数
		{
			u32_val *= 10;
			u32_val += gps_data[i++] - 0x30;
		}
	}

	if(type == LAT_TYPE)
	{
		u32_val += u32_val_div;
		u32_val /= 10;///精度，小数点后4位
		if(u32_val > 90000000)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == LONG_TYPE)
	{
		u32_val += u32_val_div;
		
		u32_val /= 10;
		if(u32_val > 180000000)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == SPEED_TYPE)
	{		
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		u32_val *= 1852;
		u32_val /= 1000;
		
		if(u32_val > 255)///节转KM/H
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == DIR_TYPE)
	{
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		u32_val /= 2;///1代表2度
		
		if(u32_val > 180)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else if(type == AMP_TYPE)
	{
		for(i=0;i<j;i++)///取整
		{
			u32_val /= 10;
		}
		
		if(u32_val > 9999)
		{
			res = FALSE;
			goto RETURN_LAB;
		}
	}
	else
	{
		res = FALSE;
		goto RETURN_LAB;
	}
	
	*r_val = u32_val;
	res = TRUE;
RETURN_LAB:
	return res;
}

static void GpsAdd8Hour(uint8 d_t[])
{
	if(d_t[3] < 16)
	{
		d_t[3] += 8;
	}
	else
	{
		d_t[3] += 8;
		d_t[3] = d_t[3] % 24;
		
		d_t[2] += 1;
		if(d_t[2] > 28)
		{
			switch(d_t[1])
			{
				case 1:
				case 3:
				case 5:
				case 7:
				case 8:
				case 10:
				{
					if(d_t[2] > 31)
					{
						d_t[2] = 1;
						d_t[1] += 1;
					}
					break;
				}
				case 2:
				{
					if((d_t[0] % 4 == 0)&&(d_t[0] % 100 != 0))
					{
						if(d_t[2] > 29)
						{
							d_t[2] = 1;
							d_t[1] += 1;
						}
					}
					else
					{
						if(d_t[2] > 28)
						{
							d_t[2] = 1;
							d_t[1] += 1;
						}
					}
					break;
				}
				case 4:
				case 6:
				case 9:
				case 11:
				{
					if(d_t[2] > 30)
					{
						d_t[2] = 1;
						d_t[1] += 1;
					}
					break;
				}
				case 12:
				{
					if(d_t[2] > 31)
					{
						d_t[2] = 1;
						d_t[1] = 1;
						d_t[0] += 1;
					}
					break;
				}
			}
		}
	}
}

static void GpsGetGprmcGpggaInfo(uint8 gps_data[],uint8 len,uint8 type)///获取GPRMC数据
{
	uint8 i,j,dot_index[9],t_d[6],res,res_1,res_2;
	uint8 lat_len,long_len,speed_len,dir_len,sat_len,amp_len;
	uint16 tmp_amp_val;
	uint32 lat_val,long_val,speed_val,dir_val,sat_val,amp_val;
	
	#ifdef GPS_DEBUG
		char str_ch[256];
		uint8 str_len;
		(void) str_len;
	#endif

	
	i = 0;j = 0;
	while((i<len)&&(j<11))
	{
		if(gps_data[i] == ',')
		{
			dot_index[j++] = i;
		}
		i++;
	}
	
	if(j != 11)
	{
		goto RETURN_LAB;
	}
	
	if(type == RMC_TYPE)
	{
		if((gps_data[0] != ',')&&(dot_index[0] >= 6)&&
		   (gps_data[dot_index[7]+1] != ',')&&((dot_index[8]-dot_index[7]) >= 6))///提取日期时间
		{
			res_1 = GpsGetTime(gps_data,t_d+3);
			res_2 = GpsGetDate(gps_data+dot_index[7]+1,t_d);
			
			if(res_1 && res_2)
			{
				GpsAdd8Hour(t_d);
								
				g_nss_info->Time.year = t_d[0];
				g_nss_info->Time.month = t_d[1];
				g_nss_info->Time.day = t_d[2];
				g_nss_info->Time.hour = t_d[3];
				g_nss_info->Time.minute = t_d[4];
				g_nss_info->Time.second = t_d[5];
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"%02d年%02d月%02d日 %02d时%02d分%02d秒\t",t_d[0],t_d[1],t_d[2],t_d[3],t_d[4],t_d[5]);
					printf(str_ch);
				#endif
			}
		}

		if(type == RMC_TYPE)
		{
			if(gps_data[dot_index[0]+1] != 'A')
			{
				g_nss_info->Status = 1;
			}
			else
			{
				g_nss_info->Status = 0;
			}
		}
/*
		if(g_nss_info->Status == 1)
		{
			goto RETURN_LAB;
		}*/
		
		lat_len = dot_index[2]-dot_index[1]-1;
		long_len = dot_index[4]-dot_index[3]-1;
		if((gps_data[dot_index[1]+1] != ',')&&(lat_len > 0)&&
		   (gps_data[dot_index[3]+1] != ',')&&(long_len > 0))
		{
			if(((gps_data[dot_index[2]+1] == 'N')||(gps_data[dot_index[2]+1] == 'S'))&&
			   ((gps_data[dot_index[4]+1] == 'E')||(gps_data[dot_index[4]+1] == 'W')))///提取纬度与经度
			{
				res_1 = GpsCharFloatDataToHex(gps_data+dot_index[1]+1,lat_len,LAT_TYPE,&lat_val);
				res_2 = GpsCharFloatDataToHex(gps_data+dot_index[3]+1,long_len,LONG_TYPE,&long_val);
				
				if(res_1 && res_2)
				{
					if(gps_data[dot_index[2]+1] == 'N')
					{
						g_nss_info->SNhemisphere = 1;
					}
					else
					{
						g_nss_info->SNhemisphere = 0;
					}
					
					if(gps_data[dot_index[4]+1] == 'E')
					{
						g_nss_info->EWhemisphere = 0;
					}
					else
					{
						g_nss_info->EWhemisphere = 1;
					}

					ToBcdLat(g_nss_info->Latitude, gps_data+dot_index[1]+1, lat_len);
					ToBcdLng(g_nss_info->Longitude, gps_data+dot_index[3]+1, long_len);
					
					#ifdef GPS_DEBUG
						str_len = sprintf(str_ch,"%ld%c %ld%c\t",lat_val,gps_data[dot_index[2]+1],long_val,gps_data[dot_index[4]+1]);
						printf(str_ch);
					#endif
					
				}
			}
		}
		
		speed_len = dot_index[6]-dot_index[5]-1;
		if((gps_data[dot_index[5]+1] != ',')&&(speed_len > 0))///提取速度
		{
			res = GpsCharFloatDataToHex(gps_data+dot_index[5]+1,speed_len,SPEED_TYPE,&speed_val);
			if(res)
			{
				ToBcdSpeed(g_nss_info->Speed, gps_data+dot_index[5]+1, speed_len);
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"速度：%03d\t",speed_val);
					printf(str_ch);
				#endif
			}
		}
		
		dir_len = dot_index[7]-dot_index[6]-1;
		if((gps_data[dot_index[6]+1] != ',')&&(dir_len > 0))///提取航向
		{
			res = GpsCharFloatDataToHex(gps_data+dot_index[6]+1,dir_len,DIR_TYPE,&dir_val);
			if(res)
			{
				ToBcdDirection(g_nss_info->Direction, gps_data+dot_index[6]+1, dir_len);
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"航向：%03d 度\t",dir_val*2);
					printf(str_ch);
				#endif
			}
		}
	}
	else if(type == GGA_TYPE)
	{
		sat_len = dot_index[6]-dot_index[5]-1;
		if((gps_data[dot_index[5]+1] != ',')&&(sat_len > 0)&&(sat_len <= 2))///提取卫星个数
		{
			res = IsValidNum(gps_data+dot_index[5]+1,sat_len);
			if(res)
			{
				sat_val = 0;
				for(i=0;i<sat_len;i++)
				{
					sat_val *= 10;
					sat_val += gps_data[dot_index[5]+1+i] - 0x30;
				}

				g_nss_info->PlanetNum = sat_val;
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"可用卫星数: %02d\t",sat_val);
					printf(str_ch);
				#endif
			}
		}
		else
		{
			g_nss_info->PlanetNum = 0;
		}
		
		amp_len = dot_index[8]-dot_index[7]-1;
		if((gps_data[dot_index[7]+1] != ',')&&(amp_len > 0))///提取海拔
		{
			ToBcdAltitude(g_nss_info->Altitude, gps_data+dot_index[7]+1,amp_len);
			
			tmp_amp_val = 0x0000;
			i = 1;
			
			if(gps_data[dot_index[7]+1] == '-')
			{
				tmp_amp_val = 0x8000;
				i = 2;
				amp_len -= 1;
			}
			res = GpsCharFloatDataToHex(gps_data+dot_index[7]+i,amp_len,AMP_TYPE,&amp_val);
			if(res)
			{
				amp_val = tmp_amp_val | amp_val;
				
				#ifdef GPS_DEBUG
					str_len = sprintf(str_ch,"海拔: %d\r\n",amp_val);
					printf(str_ch);
				#endif
			}
		}
	}
RETURN_LAB:
	return;
}

static uint8 GpsFrameCheck(uint8 gps_data[],uint8 len)
{
	uint8 i,res = FALSE;
	uint8 gps_check,cal_check;
	
	i = 0;
	cal_check = gps_data[i++];
	
	for(i=1;i<len-3;i++)
	{
		cal_check ^= gps_data[i];
	}

	gps_check = AsciiToHexVal(gps_data[len-2],gps_data[len-1]);
	
	if(gps_check == cal_check)
	{
		res = TRUE;
	}

	return res;
}

void UART4_IRQHandler(void)///GPS
{
	#ifdef UART4_BRIDDGE_UART1
	uint8 byte;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(UART4);
		USART_SendData(USART1, byte);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	
	}
	
	#else
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{	
		if(g_gps_struct.rx_counter >= GPS_MAX_BUF_LEN)
		{
			g_gps_struct.rx_counter -= 1;
		}
		g_gps_struct.rx_buf[g_gps_struct.rx_counter++] = USART_ReceiveData(UART4);
		g_gps_struct.rx_timeout_decounter = 3;
	}
	#endif
}

static void GpsUartParaInit(void)
{
	g_gps_struct.rx_counter = 0;
	g_gps_struct.rx_timeout_decounter = 0;
	g_gps_struct.rx_done_flag = INVALID_VAL_55;
}

static uint8 GpsTryRxDone(void)
{
	static uint8 count = 0;
	
	if(g_gps_struct.rx_timeout_decounter > 0)
	{
		g_gps_struct.rx_timeout_decounter -= 1;
	}
	
	if((g_gps_struct.rx_timeout_decounter == 0)&&(g_gps_struct.rx_counter > 0))
	{
		if(skip_broken_result == FALSE)
		{
			skip_broken_result = TRUE;
			GpsUartParaInit();
		}
		else
		{
			//g_gps_struct.rx_done_flag = VALID_VAL_AA;
			skip_broken_result = FALSE;

			g_gps_recv_len = g_gps_struct.rx_counter;
			MemCpy(gps_rx_data, g_gps_struct.rx_buf, g_gps_recv_len);

			GpsUartParaInit();

			g_gps_recv_ok = TRUE;
		
			return 0;
		}
	}

	// check ant status
	if(count % 10 == 0)
	{
		if(GET_GPS_ANT_STATUS() == FALSE)
		{
			gps_ant_cut = FALSE;
			#ifdef GPS_DEBUG
			//printf("gps ant ok\r\n");
			#endif
		}
		else
		{
			gps_ant_cut = TRUE;
			#ifdef GPS_DEBUG
			//printf("gps ant cut\r\n");
			#endif
		}

		count = 0;
	}

	count++;
	return 1;
}

// 0 : normal  1 : cut
uint8 GetNssAntStatus(void)
{
	if(GET_GPS_ANT_STATUS() == FALSE){
		gps_ant_cut = FALSE;
	}
	else{
		gps_ant_cut = TRUE;
	}
	
	if(gps_ant_cut)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// interface for weichai
uint8 NssInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOD, ENABLE);
	
    //串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_UART4);     //GPIOA0复用为UART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_UART4);    //GPIOA1复用为UART4
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; ///CT_GPS, GPS电源控制
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	ON_GPS_PWR();

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; ///GPS天线剪线报警
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOF, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
			
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	USART_Cmd(UART4, ENABLE);
	
	return 0;
}

/**
 * uint8 GetCurrentPosition(NSS_INFO *NssInfo)
 *
 * prama: 
 * NssInfo:  infomation
 *
 * return:
 * 0: 定位信息更新完成  1:定位信息更新未完成  2: 模块通讯故障
 *
 */
uint8 GetCurrentPosition(NSS_INFO *NssInfo)
{
	uint8 res;
	uint16 mat_index,i,j;

	bool valid = FALSE;
	
	if(NssInfo == NULL)
	{
		return 1;
	}

	GpsTryRxDone();

	RamZero((uint8*)NssInfo, sizeof(NSS_INFO));
	NssInfo->Status = 1;	// set invalid first

	if(g_gps_recv_ok == FALSE)
	{
		return 1;
	}
	
	if((g_gps_recv_len > 0)&&(g_gps_recv_len <= GPS_MAX_BUF_LEN))
	{
		g_nss_info = NssInfo;
		
		#ifdef GPS_DEBUG
			for(i=0; i < g_gps_recv_len; i++)
			{
				printf("%c", gps_rx_data[i]);
			}
			printf("\r\n");
		#endif
		
		i = 0;
		j = 0;
		while(g_gps_recv_len > 3)///除去$,CH,CR三个字符
		{	
			while((gps_rx_data[i] != '$')&&(i<g_gps_recv_len))
			{
				i++;
			}
			if(i == g_gps_recv_len)
			{
				break;
			}
			
			j=0;
			while((gps_rx_data[i+j] != 0x0a)&&((i+j)<g_gps_recv_len))
			{
				j++;
			}
			if((i+j) == g_gps_recv_len)
			{
				break;
			}
			
			res = GpsFrameCheck(gps_rx_data+i+1,j-2);///帧校验
			
			if(res)
			{
				mat_index = SubMatch("$GNRMC,",StrLen("$GNRMC,",0),gps_rx_data+i,j);
				if(mat_index > 0)
				{
					GpsGetGprmcGpggaInfo(gps_rx_data+i+mat_index,j-mat_index,RMC_TYPE);///GPRMC,GPGGA处理
					//printf("match $GNRMC\r\n");
					valid = TRUE;
					break;///RMC处理后，直接结束
				}
				
				if(mat_index > 0)
				{
					i += j;
					continue;
				}
				
				mat_index = SubMatch("$GNGGA,",StrLen("$GNGGA,",0),gps_rx_data+i,j);
				if(mat_index > 0)
				{
					GpsGetGprmcGpggaInfo(gps_rx_data+i+mat_index,j-mat_index,GGA_TYPE);///GPRMC,GPGGA处理
					//printf("match $GNGGA\r\n");
					//break;///GGA处理后，直接结束
				}
			}
			i += j;
		}

		g_gps_recv_len = 0;
		g_gps_recv_ok = FALSE;

		if(valid == TRUE)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}

	g_gps_recv_len = 0;
	g_gps_recv_ok = FALSE;
	
	return 1;
}

// 只断电
uint8 NssResetStart(void)
{
	OFF_GPS_PWR();	
	
	return 0;
}

// 只上电
uint8 NssResetComplete(void)
{
	ON_GPS_PWR();
	
	return 0;
}

