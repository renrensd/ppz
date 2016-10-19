#include <stm32f4xx.h>
#include <stdio.h>
#include "sdio_sd.h"
#include "ff.h"
#include "diskio.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "calibration.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "mcu_periph/sys_time.h"
#ifdef CALIBRATION_OPTION
#include "subsystems/datalink/downlink.h"
#include "uplink_ac.h"

#ifdef WDG_OPTION
#include "wdg.h"
#endif

static FIL fdata;
static FIL fraw;
FIL ffil;
FATFS fs;           
FRESULT res;                 
DIR dirs;
FILINFO finfo;

struct CALI_INFO cali_info;

static int length_after_filter = 0;
static double p0[6]={0,0,0,0,0,0};
//static double x[9];
//static double theta[6];
static double err=0;
static double err_last = 0;
static double gradient[6];
static int noise1[3];
static int temp1[20][3];
static int byte = 0;


static void cali_process(void);
static void cali_magraw_txt_creat(void);
static void cali_magraw_begin(void);
static void cali_magraw_end(void);
static void cali_mean(int n[20][3], int length, int mean_of_vec[3]);
static int norm(int *n);
static double norm_f(double *n);
static void cut_vector_file(FIL fr, int min, int max, int out1[20][3]);
static void std(int n[20][3], int length, int out[3]);
static int noise_filter_file(int window_size,int noise_threshold);
static int *max_fun_file(FIL fd ,int length);
static int *min_fun_file(FIL fd ,int length);
static double *get_min_max_guess_file(int scale);
static void scale_measurements_file( double *p);
static void err_fun(void);
#if 0
static void read_log(void);
static int StringFind(const char *pSrc, const char *pDst);
#endif


/***********************************************************************
* Name : StringFind
* Description : find the position of string B in A.
* Parameter : String A and B
* Returns : int index
******/
static int StringFind(const char *pSrc, const char *pDst)
{
	int i, j;
	for (i=0; pSrc[i]!='\0'; i++)
	{
		if(pSrc[i]!=pDst[0])
		continue;		
		j = 0;
		while(pDst[j]!='\0' && pSrc[i+j]!='\0')
		{
			j++;
			if(pDst[j]!=pSrc[i+j])
			break;
		}
		if(pDst[j]=='\0')
			return i;
	}
	return -1;
}

#if 0
/***********************************************************************
* Name : read_log
* Description : read the log data and strip the mag_data.
* Parameter : void
* Returns :void
******/
static void read_log(void)
{

	char pBuf[100];
	int len;
	char *data_new;
	char string[20];
	int ptr;
	int counter;
	int i,j;

	i=0;
	j=0;

	//printf("\n This is least_squar algorithm for caliberation!\n");
	res = f_open(&fraw,"16_01_02__10_32_18.data",FA_READ);
    

    f_open(&fdata,"save.txt",FA_CREATE_ALWAYS); //create new file always.
	f_close(&fdata);
	f_open(&fdata,"save.txt",FA_WRITE);

	if(res != FR_OK)
	{
		//printf("Cannot open\n");
	}
	else
	{
		//printf(" openned\n");
        len = fraw.fsize;                        //Total file size
        //printf("   len = %d  \n",len);         
		strcpy(string, "IMU_MAG_RAW");          //Sign of storage 
		while(f_gets(pBuf,sizeof(pBuf),&fraw )) 
		{	
			ptr = StringFind(pBuf,string);	     //Comparison of signs
			if((ptr+1)!=0)
			{
				data_new = strtok(pBuf+ptr," ");/*find mag data and transfer to int*/ 
				j = 0;
			
				for(j = 0;(data_new!=NULL);j++)
				{
					data_new = strtok(NULL," ");         //Effective data separation
					if((data_new==NULL)||(data_new==""))
					{
						break;
					}
                    if(j<2)
                        f_printf(&fdata,"%s ",data_new);   //Effective data storage
                    else
                    {
                        f_printf(&fdata,"%s",data_new);    //Effective data storage
                        f_printf(&fdata,"\0");
                    }
				}
				i++;
			}
		}
	}
	f_close(&fdata);
	counter = i;
    cali_info.mag_txt_len = counter ;          
	f_close(&fraw);
	//printf("there are %d data for mag \n", i); 
}
#endif

static void cali_magraw_txt_creat(void)
{
	f_open(&fraw, "save.txt", FA_CREATE_ALWAYS);
	f_close(&fraw);
}

static void cali_magraw_begin(void)
{
    f_open(&fraw,"save.txt",FA_WRITE);
}

static void cali_magraw_end(void)
{
    f_close(&fraw);
}

void cali_magraw_to_txt(int32_t x1, int32_t y1, int32_t z1)
{
	if(cali_info.mag_entry_flag == TRUE)
	{
		cali_info.mag_cnt++;
		if(cali_info.mag_cnt >= CALI_MAG_RAW_CNT)
		{
			cali_info.mag_cnt = 0;
			f_printf(&fraw,"%D %D %D\n",x1,y1,z1);
			cali_info.mag_txt_len++;
		}
	}
}

/***********************************************************************
* Name : cali_mean
* Description : calculate the average value of a each 1D vector in a 2D vec. 
* Parameter : int n[][], int length
* Returns : int cali_mean[]
******/

static void cali_mean(int n[20][3], int length, int mean_of_vec[3])
{
	int i,j;
	mean_of_vec[0]=0;
    mean_of_vec[1]=0;
    mean_of_vec[2]=0;
	for(j=0;j<3;j++)
	{
		for(i=0;i<length;i++)
		{
			mean_of_vec[j] += n[i][j];
		}
	}
	mean_of_vec[0] = mean_of_vec[0]/(length);
	mean_of_vec[1] = mean_of_vec[1]/(length);
	mean_of_vec[2] = mean_of_vec[2]/(length);
}


/***********************************************************************
* Name : norm
* Description : get the norm2 of 1d vector
* Parameter : int vector
* Returns : int the norm
******/
static int norm(int *n)
{
		
	int norm = 0;
	int j;
	
	for(j=0;j<3;j++)
	{
		norm += (n[j])*(n[j]);
	}

	norm = (int)sqrt((double)norm);

    return norm;
}

/***********************************************************************
* Name : norm_f
* Description : get norm2 in double form 
* Parameter : vector
* Returns : norm
******/
static double norm_f(double *n)
{

		double norm = 0;
		int j;

		for(j=0;j<3;j++)
		{
			norm += (n[j])*(n[j]);
		}

		norm = sqrt((double)norm);

	return norm;
}

/***********************************************************************
* Name : cut_vector
* Description : get part of 2D vector based on size of window
* Parameter : start min, end max, vector n.
* Returns : cutted vector
******/
static void cut_vector_file(FIL fr, int min, int max, int out1[20][3])
{
	int i=0;
	int j=0;
	char pBuf[30];
	char *data_new;
    
        if(min==0)
        {
            for(i=0;i<(max-min);i++)
            {
                f_gets(pBuf,sizeof(pBuf),&fr );
                data_new = strtok(pBuf," ");
                for(j=0;j<3;j++)
                {
                    out1[i][j] = atoi(data_new);
                    data_new = strtok(NULL," ");
                }	
            }	
        }   
        else
        {
            for(i=0;i<(max-min-1);i++)
            {
                out1[i][0]=out1[i+1][0];
                out1[i][1]=out1[i+1][1];
                out1[i][2]=out1[i+1][2];
            }
            f_gets(pBuf,sizeof(pBuf),&fr );
            data_new = strtok(pBuf," ");
            for(j=0;j<3;j++)
            {
                out1[max-min-1][j] = atoi(data_new);
                data_new = strtok(NULL," ");
            }
        }      
}
/***********************************************************************
* Name : std
* Description : get the standard variation of vector. 
* Parameter : vector and length
* Returns : standard variation. 
******/
static void std(int n[20][3], int length, int out[3])
{
    int data[10][3];
    int i=0;
	int j=0;
	int miu[3] = {0,0,0};
	int temp;
	for(i=0;i<length;i++)
	{
 		for(j=0;j<3;j++)
		{
			cali_mean(n, length,miu);
			temp = ((n[i][j]-miu[j])*(n[i][j]-miu[j]));
			data[i][j] = (int)((double)temp);
		}	
	}
    cali_mean(data,length,miu);
	out[0] = (int)sqrt((double)miu[0]);
	out[1] = (int)sqrt((double)miu[1]);
	out[2] = (int)sqrt((double)miu[2]);

}

/***********************************************************************
* Name : noise_filter
* Description : filte the data with specified window size and noise threshold
* Parameter : vector, windows size, threshold
* Returns : filted data. 
******/

static int noise_filter_file(int window_size,int noise_threshold)
{
	int i;
	char pBuf[30];
 
    int j=0;
	int noi[3]={0,0,0};
	int ii=0;
    
    res = f_open(&fraw,"save.txt",FA_READ);          //Open the AMG data
    
	f_open(&fdata,"mag_filter.txt",FA_CREATE_ALWAYS);
	f_close(&fdata); 
    f_open(&fdata,"mag_filter.txt",FA_WRITE);

    f_lseek(&fraw,0);
    for(j=0;j<10;j++)                       //Remove the first 10 sets of data
        f_gets(pBuf,sizeof(pBuf),&fraw );
	for(i=window_size; i<(cali_info.mag_txt_len-window_size); i++)
	{
        cut_vector_file(fraw,i-window_size, i+window_size,temp1);   //The original data read
		std(temp1,window_size,noise1);
		noi[0]=noise1[0];
		noi[1]=noise1[1];
		noi[2]=noise1[2];
		f_gets(pBuf,sizeof(pBuf),&fraw);
		if(norm(noi)<noise_threshold)
		{
			f_puts(pBuf, &fdata);       //Filter data storage
		}
		ii++;
	}
	f_printf(&fdata,"\0");
    f_close(&fdata);
    f_close(&fraw);
    byte = 0;
	length_after_filter = ii;
	//printf("there are %d data after filtering\n",ii);
	return 1;
}

/***********************************************************************
* Name : max_fun
* Description : get max 1D vector
* Parameter : 2D vector, length
* Returns : 1D vector. 
******/	
static int *max_fun_file(FIL fd ,int length)
{
	int i=0;
	char *data_new;
	char pBuf[30];
	static int maximum[3] = {0,0,0};

    f_lseek(&fd,0);
	for(i=0;i<length;i++)
	{
		f_gets(pBuf,sizeof(pBuf),&fd );
		data_new = strtok(pBuf," ");
		if(data_new!=NULL)
        {
            if(maximum[0]<atoi(data_new))
            {
                maximum[0]=atoi(data_new);
            }
		}
		data_new = strtok(NULL," ");
		if(data_new!=NULL)
        {
			if(maximum[1]<atoi(data_new))
			{
				maximum[1]=atoi(data_new);
			}
		}
		data_new = strtok(NULL," ");
		if(data_new!=NULL)
        {
			if(maximum[2]<atoi(data_new))
			{
				maximum[2]=atoi(data_new);
			}
		}
	}
	return maximum;
}

/***********************************************************************
* Name : min_fun
* Description : get minimum vector
* Parameter : mea[][],length
* Returns :minimum vector. 
******/
static int *min_fun_file(FIL fd ,int length)
{
	int i=0;
	char *data_new;
	char pBuf[20];
	static int maximum[3] = {0,0,0};

    f_lseek(&fd,0);
	for(i=0;i<length;i++)
	{
		f_gets(pBuf,sizeof(pBuf),&fd );
		data_new = strtok(pBuf," ");
		if(data_new!=NULL)
        {
            if(maximum[0]>atoi(data_new))
            {
                maximum[0]=atoi(data_new);
            }
		}
		data_new = strtok(NULL," ");
		if(data_new!=NULL)
        {
			if(maximum[1]>atoi(data_new))
			{
				maximum[1]=atoi(data_new);
			}
		}
		data_new = strtok(NULL," ");
		if(data_new!=NULL)
        {
			if(maximum[2]>atoi(data_new))
			{
				maximum[2]=atoi(data_new);
			}
		}
	}
	return maximum;
}

/***********************************************************************
* Name : get_min_max_guess
* Description : calculate initial parameters. 
* Parameter : data, and scale
* Returns : parameters
******/
static double *get_min_max_guess_file(int scale)
{
	int *max_meas;
	int *min_meas;
	int range[3]= {0,0,0};
	int i;
	double n[3]={0,0,0};
	double sf[3] = {0,0,0};
	static double min_max_guess[6]={0,0,0,0,0,0};
	int maxi[3]={0,0,0};
	int mini[3]={0,0,0};
    
    f_open(&fdata,"mag_filter.txt",FA_READ); 

	for(i=0;i<3;i++)
    {
		max_meas = max_fun_file(fdata,length_after_filter);
		maxi[i]=max_meas[i];
		min_meas = min_fun_file(fdata,length_after_filter);
		mini[i]=min_meas[i];
		range[i] = maxi[i]-mini[i];
		n[i] = (double)(maxi[i]+mini[i])/2;
		sf[i] = 2*scale/(double)range[i];
	}	

	p0[0]=n[0];
	p0[1]=n[1];
	p0[2]=n[2];
	p0[3]=sf[0];
	p0[4]=sf[1];
	p0[5]=sf[2];
	f_close(&fdata);
    
	return min_max_guess;
}

/***********************************************************************
* Name : scale_measurement
* Description :  scale the measurement with parameters
* Parameter : meas[][], parameter
* Returns :void
******/
static void scale_measurements_file( double *p)
{
	int i=0;
	int j=0;
	double sm[3]={0,0,0};
	int m[3]={0,0,0};
	double h;
	double e;
	double dp1;
	double dp2;
	double dp3;
	double dp4;
	double dp5;
	double dp6;
	int counter=0;
	char *data_new;
	char pBuf[30];

	gradient[0]=0;
	gradient[1]=0;
	gradient[2]=0;
	gradient[3]=0;
	gradient[4]=0;
	gradient[5]=0;
    
    f_open(&fdata,"mag_filter.txt",FA_READ);
    f_lseek(&fdata,0);
	for(i=0; i<length_after_filter;i++)
	{

		f_gets(pBuf,sizeof(pBuf),&fdata);
		data_new = strtok(pBuf," ");
		for(j=0;j<3;j++)
		{
			if(data_new!=NULL)
			{
				m[j] = atoi(data_new);
				sm[j] = ((double)m[j]-p[j])*p[j+3];
			}
			else
			{
				e = 0;
				goto loop;
			}
			data_new = strtok(NULL," ");
		}
		counter++;
		h = norm_f(sm);
        err += (1 - h)*(1 - h); 
		e = h-1;
		
		dp1 = (0.5/h)*(-2*m[0]*p[3]*p[3]+2*p[0]*p[3]*p[3]);
		dp2 = (0.5/h)*(-2*m[1]*p[4]*p[4]+2*p[1]*p[4]*p[4]);
		dp3 = (0.5/h)*(-2*m[2]*p[5]*p[5]+2*p[2]*p[5]*p[5]);
		dp4 = 2*p[3]*((m[0]-p[0])*(m[0]-p[0]));
		dp5 = 2*p[4]*((m[1]-p[1])*(m[1]-p[1]));
		dp6 = 2*p[5]*((m[2]-p[2])*(m[2]-p[2]));
		
     /*9250 parameter*/  
     //When the parameter exceeds a certain value will lead to the learning process can not converge   
     #if 0
        gradient[0] += 10000000*e*dp1; // This parameter determines the convergence rate of P0[0], the greater the faster convergence
		gradient[1] += 10000000*e*dp2; //This parameter determines the convergence rate of P0[1], the greater the faster convergence
		gradient[2] += 10000000*e*dp3; //This parameter determines the convergence rate of P0[2], the greater the faster convergence
		gradient[3] += 0.001*e*dp4;  //This parameter determines the convergence rate of P0[3], the greater the faster convergence
		gradient[4] += 0.001*e*dp5;  //This parameter determines the convergence rate of P0[4], the greater the faster convergence
		gradient[5] += 0.001*e*dp6;  //This parameter determines the convergence rate of P0[5], the greater the faster convergence
    #endif  
   /*5983 parameter */
   
		gradient[0] += 35000000*e*dp1;
		gradient[1] += 35000000*e*dp2;
		gradient[2] += 35000000*e*dp3;
		gradient[3] += 0.0005*e*dp4;
		gradient[4] += 0.0005*e*dp5;
		gradient[5] += 0.0005*e*dp6;

		loop: ;
	}
    f_close(&fdata);
}

/***********************************************************************
* Name : err_fun
* Description : calculate sum square error
* Parameter : void	
* Returns : void
******/
static void err_fun(void)
{
	
	scale_measurements_file( p0);
	err = err/length_after_filter; 
    
    p0[0] -= 0.01*(gradient[0]/length_after_filter);  // This parameter determines the convergence rate of the overall study,
	p0[1] -= 0.01*(gradient[1]/length_after_filter);  // the greater the faster convergence parameters,
	p0[2] -= 0.01*(gradient[2]/length_after_filter);  // The parameters not greater than 0.01
	p0[3] -= 0.01*(gradient[3]/length_after_filter);
	p0[4] -= 0.01*(gradient[4]/length_after_filter);
	p0[5] -= 0.01*(gradient[5]/length_after_filter);
    
}

/***********************************************************************
* Name : calib
* Description : integrate all the functons above, get the final parameter
* Parameter : void
* Returns :void
******/
static void cali_process(void)
{
    int i=0;
	uint32_t cali_init_time;
	uint32_t cali_cur_time;
	
	struct option 
	{
		int sensor_ref;
		int sensor_res;
		int noise_window;
		int noise_threshold;
	} opt;

	int iter=0;
	opt.sensor_ref=1;
	opt.sensor_res=11;
	opt.noise_window=10;
	opt.noise_threshold=1000;
	//read_log();                 //read the log data and strip the mag_data.
    
	noise_filter_file( opt.noise_window, opt.noise_threshold);   // filte the data with specified window size and noise threshold

    get_min_max_guess_file(opt.sensor_ref);      // calculate initial parameters.
    //printf("P0 is %f  %f  %f  %f  %f  %f\n",p0[0],p0[1],p0[2],p0[3],p0[4],p0[5]);

	scale_measurements_file( p0);    //scale the measurement with parameters
    cali_init_time = get_sys_time_msec();
	while(1)
	{ 
		cali_cur_time = get_sys_time_msec();
		if( cali_cur_time - cali_init_time > CALI_MAG_TIMEROUT )
		{
			cali_info.mag_state = CALI_MAG_STATE_FINISHED_FAIL;
			xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
			DOWNLINK_SEND_CALIBRATION_AC_RC_STATE(DefaultChannel, DefaultDevice, &cali_info.mag_state);
			return;
		}
		err_last = err;
		err_fun();                                 //calculate sum square error
        
		//printf("err is %f err_last is %f \n",err,err_last);
		//printf("P0 is %f  %f  %f  %f  %f  %f\n",p0[0],p0[1],p0[2],p0[3],p0[4],p0[5]);
		iter++;
		if((((err-err_last)<=0.000001)&&((err-err_last)>=-0.000001)))  // 0.000001  Decided to withdraw from the process of learning threshold
		{
			break;
		}
	}
	//printf("caliberation is done, caliberated p is \n %f  %f  %f  %f  %f  %f\n",p0[0],p0[1],p0[2],p0[3],p0[4],p0[5]);
	//printf("precision is %f   after  %d  iterations"  ,1-err,iter);
    
    for(i=0;i<3;i++)          //Offset four to five homes in rounding
    {
        if(p0[i]>0)
            p0[i]=p0[i]+0.5;
        else
            p0[i]=p0[i]-0.5;
    }

	p0[3] *= 2048;
	p0[4] *= 2048;
	p0[5] *= 2048;

	cali_info.mag_txt_len = 0;
	imu.mag_neutral.x = p0[0];
	imu.mag_neutral.y = p0[1];
	imu.mag_neutral.z = p0[2];
	imu.mag_sens.x = p0[3];
	imu.mag_sens.y = p0[4];
	imu.mag_sens.z = p0[5];
	autopilot_mag_cali_store();
	cali_info.mag_state = CALI_MAG_STATE_FINISHED_SUCCESS;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
	DOWNLINK_SEND_CALIBRATION_AC_RC_STATE(DefaultChannel, DefaultDevice, &cali_info.mag_state);
    //printf("MAG_X_NEUTRAL = %d \n",(int)p0[0]);
    //printf("MAG_Y_NEUTRAL = %d \n",(int)p0[1]);
    //printf("MAG_Z_NEUTRAL = %d \n",(int)p0[2]);
    //printf("MAG_X_SENS = %f \n",p0[3]*2048);
    //printf("MAG_Y_SENS = %f \n",p0[4]*2048);
    //printf("MAG_Z_SENS = %f \n",p0[5]*2048);
   
}

void sd_fatfs_init(void)
{     
    //SD_Init();
    res=f_mount(0, &fs);
	cali_info.mag_entry_flag = FALSE;
	cali_info.mag_txt_len = 0;
	cali_info.mag_state = CALI_MAG_STATE_INIT;
}

/***********************************************************************
* FUNCTION    : cali_mag_begin
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void cali_mag_begin(void)
{
	#ifdef WDG_OPTION
	wdg_enable_systick_feed();
	#endif
	
	if( (cali_info.mag_entry_flag == FALSE) && (cali_info.mag_state == CALI_MAG_STATE_INIT) )
	{
		cali_magraw_txt_creat();
		cali_magraw_begin();
		cali_info.mag_entry_flag = TRUE;
	}
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
	DOWNLINK_SEND_CALIBRATION_AC_RC_STATE(DefaultChannel, DefaultDevice, &cali_info.mag_state); 
}

/***********************************************************************
* FUNCTION    : cali_mag_end
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void cali_mag_end(void)
{
	cali_info.mag_state = CALI_MAG_STATE_WAIT_FINISHED;
	xbee_tx_header(XBEE_NACK,XBEE_ADDR_RC);
	DOWNLINK_SEND_CALIBRATION_AC_RC_STATE(DefaultChannel, DefaultDevice, &cali_info.mag_state);
	if(cali_info.mag_entry_flag == TRUE)
	{
		cali_info.mag_entry_flag = FALSE;
		cali_magraw_end();
		cali_process();
	}

	#ifdef WDG_OPTION
	wdg_disable_systick_feed();
	#endif
}

/***********************************************************************
* FUNCTION    : cali_mag_state_init
* DESCRIPTION : 
* INPUTS      : none
* RETURN      : none
***********************************************************************/
void cali_mag_state_init(void)
{
	cali_info.mag_state = CALI_MAG_STATE_INIT;
	cali_info.mag_entry_flag = FALSE;
	cali_info.mag_txt_len = 0;
}

void sd_write_file_fault(char *name, uint32_t data, uint8_t flag)
{
	uint32_t br;
	if(flag == 1)
	{
		f_open(&ffil, "fault.txt", FA_CREATE_NEW);
		f_close(&ffil);
		f_open(&ffil,"fault.txt",FA_WRITE);
		br = ffil.fsize;
		f_lseek(&ffil,br);
		f_printf(&ffil,"%x\n",0);
		f_printf(&ffil,"%s   \t",name);
		f_printf(&ffil,"%x\n",data);
	}
	else if(flag == 0)
	{
		f_printf(&ffil,"%s   \t",name);
		f_printf(&ffil,"%x\n",data);
	}
	else if(flag == 2)
	{
		f_printf(&ffil,"%s   \t",name);
		f_printf(&ffil,"%x\n",data);
		f_close(&ffil);
	}
}

#endif	/* CALIBRATION_OPTION */


