#include "gps.h" 								   							   

timestamped_gps mobile_buffer[BUFFER_SIZE];
uint8_t buffer_head = 0;      // ������������
uint8_t buffer_count = 0;     // ��Ч��������

//������ֻ��ѧϰʹ��
//ALIENTEK STM32F103C8T6������	
char SetTimeZone[25] = {0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xf1,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c};

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p,*p1,dx;
	uint8_t len,i,j,slx=0;
	uint8_t posx;   	 
	p=buf;
	p1=(uint8_t*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//�õ�GPGSV������
	posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(uint8_t*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
			else break;
			slx++;	   
		}   
 		p=p1+1;//�л�����һ��GPGSV��Ϣ
	}   
}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
//	p1=(uint8_t*)strstr((const char *)buf,"$GPGGA");
	p1=(uint8_t*)strstr((const char *)buf,"GGA");
	posx=NMEA_Comma_Pos(p1,9);							
						
	if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx); //�õ����θ߶�
	posx=NMEA_Comma_Pos(p1,7);								
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx);  //�õ�������Ŀ
}
//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
	uint8_t i;   
	p1=(uint8_t*)strstr((const char *)buf,"GSA");
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf,char timezone)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	float rs;  
	uint32_t temp;
	int itemp;
	p1=(uint8_t*)strstr((const char *)buf,"RMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//�õ�UTCʱ��,ȥ��ms
		
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	
	}
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{	
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF)
	{
		gpsx->nshemi=*(p1+posx);
	}			 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF)
	{

		gpsx->ewhemi=*(p1+posx);		 
	}	
	posx=NMEA_Comma_Pos(p1,7);								//�õ�����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		temp = temp*10*1.852;
		itemp = temp;
		gpsx->speed = itemp;
	} 

	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		gpsx->utc.date=temp/10000;
		gpsx->utc.month=(temp/100)%100;
		gpsx->utc.year=2000+temp%100;	
	}
}
//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;    
	p1=(uint8_t*)strstr((const char *)buf,"$GPVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//ȷ������1000��
	}
}  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
gps_msg last_valid_data;

void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV����
	NMEA_GPGGA_Analysis(gpsx,buf);	//GPGGA���� 	
	NMEA_GPGSA_Analysis(gpsx,buf);	//GPGSA����
	NMEA_GPRMC_Analysis(gpsx,buf,SetTimeZone[20]);	//GPRMC����
	NMEA_GPVTG_Analysis(gpsx,buf);	//GPVTG����
	
}

void gps_read(nmea_msg *gpsx,nvidia_packet *gps_send)
{
	// �ڷ������ݵĵط��޸�Ϊ��
	gps_send->data.start_marker = 0xAA55;
	
	// ʱ�䴦��
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
	
	
	// γ�ȴ���
	if (gpsx->latitude != 0 ) {
			last_valid_data.latitude = gpsx->latitude; // ��Чʱ����
	}
	gps_send->data.latitude = last_valid_data.latitude;  // ʼ��ʹ��������Чֵ
	// ���ȴ���
	if (gpsx->longitude != 0 ) {
			last_valid_data.longitude = gpsx->longitude;
	}
	gps_send->data.longitude = last_valid_data.longitude;
}

//GPSУ��ͼ���
//buf:���ݻ������׵�ַ
//len:���ݳ���
//cka,ckb:����У����.
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
    // У��������Ч�ԣ�������ʼ��־��
    if (msg->start_marker != 0xAA55) return;

    // ������ʱ���
    uint32_t new_stamp = TIME_TO_STAMP(
        msg->hour, msg->min, msg->sec, msg->ms100
    );
    // ����Ƿ��ظ�
    if (buffer_count > 0) {
        uint8_t latest_idx = (buffer_head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        uint32_t latest_stamp = mobile_buffer[latest_idx].stamp;
        
        if (new_stamp == latest_stamp) {
            // ʱ����ظ�����������
            return;
        }
    }
    // ����������
    mobile_buffer[buffer_head].stamp = new_stamp;
    memcpy(&mobile_buffer[buffer_head].data, msg, sizeof(gps_msg));
    
    // ����ָ��
    buffer_head = (buffer_head + 1) % BUFFER_SIZE;
    if (buffer_count < BUFFER_SIZE) {
        buffer_count++;
    } else {
        // ��������������������ݣ���ѡ�߼���
    }
}

// ��վ��֪��ʵ���꣨��Ԥ�����ã�
uint32_t base_true_lat = 4176183;
uint32_t base_true_lon = 12340897;


void process_base_station_data(gps_msg *base_msg,nvidia_msg *msg,timestamped_gps *mobile_buffer) {
    // �����վ����ʱ���
    uint32_t base_stamp = TIME_TO_STAMP(base_msg->hour, base_msg->min, 
                                      base_msg->sec, base_msg->ms100);
    
    // ���ƶ��˻���������ƥ��������1��ʱ�䵥λ��
    int8_t match_index = -1;
    for(uint8_t i=0; i<buffer_count; i++) {
        uint8_t idx = (buffer_head - buffer_count + i + BUFFER_SIZE) % BUFFER_SIZE;
        if(abs(mobile_buffer[idx].stamp - base_stamp) <= 1) {
            match_index = idx;
            break;
        }
    }
    
    if(match_index == -1) {
        // δ�ҵ�ƥ�����ݣ��ɼ�¼������ֵ����
        return;
    }
    // ִ�в��У��
    gps_msg *mobile = &mobile_buffer[match_index].data;
    msg->corrected_lat = mobile->latitude - (base_msg->latitude - base_true_lat);
    msg->corrected_lon = mobile->longitude - (base_msg->longitude - base_true_lon);
    
    // �����Ѵ������ݣ���ѡ��
    buffer_count -= (match_index + 1);
}
