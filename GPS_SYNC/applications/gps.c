#include "gps.h" 								   							   

timestamped_gps mobile_buffer[BUFFER_SIZE];
uint8_t buffer_head = 0;      // 最新数据索引
uint8_t buffer_count = 0;     // 有效数据数量

//本程序只供学习使用
//ALIENTEK STM32F103C8T6开发板	
char SetTimeZone[25] = {0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xf1,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c};

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}
//分析GPGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
//	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");
	p1=(uint8_t*)strstr((const char *)buf,"GGA");
	posx=NMEA_Comma_Pos(p1,9);							
						
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx); //得到海拔高度
	posx=NMEA_Comma_Pos(p1,7);								
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx);  //得到卫星数目
}
//分析GPGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"GSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//分析GPRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf,char timezone)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	float rs;  
	uint32_t temp;
	int itemp;
	p1=(uint8_t*)strstr((const char *)buf,"RMC");//"$GPRMC",经常有&和GPRMC分开的情况,故只判断GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	
	}
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{	
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
	if(posx!=0XFF)
	{
		gpsx->nshemi=*(p1+posx);
	}			 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	if(posx!=0XFF)
	{

		gpsx->ewhemi=*(p1+posx);		 
	}	
	posx=NMEA_Comma_Pos(p1,7);								//得到速率
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		temp = temp*10*1.852;
		itemp = temp;
		gpsx->speed = itemp;
	} 

	posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	
	}
}
//分析GPVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
gps_msg last_valid_data;

void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA解析 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA解析
	NMEA_GPRMC_Analysis(gpsx,buf,SetTimeZone[20]);	//GPRMC解析
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG解析
	
}

void gps_read(nmea_msg *gpsx,nvidia_packet *gps_send)
{
	// 在发送数据的地方修改为：
	gps_send->data.start_marker = 0xAA55;
	
	// 时间处理
	if(gpsx->utc.year != 0 && gpsx->utc.month != 0 && gpsx->utc.date != 0 && gpsx->utc.hour != 0 && gpsx->utc.min != 0 && gpsx->utc.sec != 0) {
			last_valid_data.hour = gpsx->utc.hour;
			last_valid_data.min = gpsx->utc.min;
			last_valid_data.sec = gpsx->utc.sec;
			if(last_valid_data.sec != gps_send->data.sec) {
					gpsx->utc.ms100 = 0;
			}
			last_valid_data.ms100 = gpsx->utc.ms100;
	}
	
	gps_send->data.hour = last_valid_data.hour;
	gps_send->data.min = last_valid_data.min;
	gps_send->data.sec = last_valid_data.sec;
	gps_send->data.ms100 = last_valid_data.ms100;
	
	
	// 纬度处理
	if (gpsx->latitude != 0 ) {
			last_valid_data.latitude = gpsx->latitude; // 有效时更新
	}
	gps_send->data.latitude = last_valid_data.latitude;  // 始终使用最新有效值
	// 经度处理
	if (gpsx->longitude != 0 ) {
			last_valid_data.longitude = gpsx->longitude;
	}
	gps_send->data.longitude = last_valid_data.longitude;
}

//GPS校验和计算
//buf:数据缓存区首地址
//len:数据长度
//cka,ckb:两个校验结果.
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
{
	uint16_t i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}

void save_mobile_data(nvidia_msg *msg) {
    // 校验数据有效性（例如起始标志）
    if (msg->start_marker != 0xAA55) return;

    // 生成新时间戳
    uint32_t new_stamp = TIME_TO_STAMP(
        msg->hour, msg->min, msg->sec, msg->ms100
    );
    // 检查是否重复
    if (buffer_count > 0) {
        uint8_t latest_idx = (buffer_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        uint32_t latest_stamp = mobile_buffer[latest_idx].stamp;
        
        if (new_stamp == latest_stamp) {
            // 时间戳重复，丢弃数据
            return;
        }
    }
    // 存入新数据
    mobile_buffer[buffer_head].stamp = new_stamp;
    memcpy(&mobile_buffer[buffer_head].data, msg, sizeof(gps_msg));
    
    // 更新指针
    buffer_head = (buffer_head + 1) % BUFFER_SIZE;
    if (buffer_count < BUFFER_SIZE) {
        buffer_count++;
    } else {
        // 缓存区满，覆盖最旧数据（可选逻辑）
    }
}

// 基站已知真实坐标（需预先配置）
uint32_t base_true_lat = 4176183;
uint32_t base_true_lon = 12340897;


void process_base_station_data(gps_msg *base_msg,nvidia_msg *msg,timestamped_gps *mobile_buffer) {
    // 计算基站数据时间戳
    uint32_t base_stamp = TIME_TO_STAMP(base_msg->hour, base_msg->min, 
                                      base_msg->sec, base_msg->ms100);
    
    // 在移动端缓冲区查找匹配项（允许±1个时间单位误差）
    int8_t match_index = -1;
    for(uint8_t i=0; i<buffer_count; i++) {
        uint8_t idx = (buffer_head - buffer_count + i + BUFFER_SIZE) % BUFFER_SIZE;
        if(abs(mobile_buffer[idx].stamp - base_stamp) <= 1) {
            match_index = idx;
            break;
        }
    }
    
    if(match_index == -1) {
        // 未找到匹配数据（可记录错误或插值处理）
        return;
    }
    // 执行差分校正
    gps_msg *mobile = &mobile_buffer[match_index].data;
    msg->corrected_lat = mobile->latitude - (base_msg->latitude - base_true_lat);
    msg->corrected_lon = mobile->longitude - (base_msg->longitude - base_true_lon);
    
    // 清理已处理数据（可选）
    buffer_count -= (match_index + 1);
}
