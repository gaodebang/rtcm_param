#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>

typedef enum {
    PARSE_STATE_UNINIT=0,
    PARSE_STATE_IDLE,
    PARSE_STATE_GOT_STX,
    PARSE_STATE_GOT_LEN1,
    PARSE_STATE_GOT_LEN0,
    PARSE_STATE_GOT_PAYLOAD,
    PARSE_STATE_GOT_CRC2,
    PARSE_STATE_GOT_CRC1,
    PARSE_STATE_GOT_CRC0,
}rtcm_parse_state_t;

typedef enum {
    FRAMING_INCOMPLETE=0,
    FRAMING_OK=1,
    FRAMING_BAD_CRC=2
}rtcm_framing_t;

typedef struct _rtcm_status {
    rtcm_framing_t frame_status;
    rtcm_parse_state_t parse_state;  		///< Parsing state machine
    unsigned int packet_rx_success_count;   ///< Received packets
    unsigned int parse_error;            	///< Number of parse errors
    unsigned int packet_idx;                ///< Index in current packet
    unsigned int buffer_overrun;            ///< Number of buffer overruns
}rtcm_status_t;

union uchar_ushort
{
	unsigned char no1[2];
	unsigned short no2;
};
union uchar_ulong
{
	unsigned char no1[4];
	unsigned int no2;
};

typedef struct _rtcm_message {
	unsigned char magic;
	union uchar_ushort len;
	unsigned char payload[1023];
	union uchar_ulong crc;
}rtcm_message_t;
 
//#define DEV_NAME  "/dev/ttyAMA0"
#define DEV_NAME	"/dev/ttyS13"
#define RTCM_STX	0xD3

#define CRC_MULTINOMIAL 0x01864cfb

#define CRC24_INIT	0xB740CEL
#define CRC24_POLY	0x1864CFBL

static void crc_accumulate(unsigned char data, unsigned int *crcAccum)
{
	unsigned char i;
	*crcAccum ^= (unsigned int)data << (16);
	for(i = 0; i < 8; i++)
	{
		*crcAccum <<= 1;
		if(*crcAccum & 0x1000000)
			*crcAccum ^= CRC_MULTINOMIAL;
	}
}
static void crc_update_buf(const unsigned char* pBuffer, unsigned int length, unsigned int *crcAccum)
{
	while(length--)
	{
		crc_accumulate(*pBuffer++, crcAccum);
	}
}
static void crc_init(unsigned int *crcAccum)
{
	*crcAccum = 0;
}
static unsigned int crc_calculate(const unsigned char* pBuffer, unsigned int length)
{
	unsigned int crcTmp;
	crc_init(&crcTmp);
	while (length--) 
	{
		crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}

#define _RTCM_PAYLOAD(msg) ((const char *)(&((msg)->payload[0])))
#define _RTCM_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload[0])))

rtcm_framing_t frame_char_buf(unsigned char data, rtcm_status_t *status, rtcm_message_t *rxmsg)
{
	status->frame_status = FRAMING_INCOMPLETE;
	switch(status->parse_state)
	{
		case PARSE_STATE_UNINIT:
		case PARSE_STATE_IDLE:
			if(data == RTCM_STX)
			{
				rxmsg->len.no2 = 0;
				rxmsg->magic = data;
				status->parse_state = PARSE_STATE_GOT_STX;
			}
			break;
		case PARSE_STATE_GOT_STX:
			if(data >= 0x03)
			{
				status->buffer_overrun++;
				status->frame_status = FRAMING_INCOMPLETE;
				status->parse_state = PARSE_STATE_IDLE;
			}
			else
			{
				rxmsg->len.no1[1] = data;
	      status->parse_state = PARSE_STATE_GOT_LEN1;
			}
			break;
		case PARSE_STATE_GOT_LEN1:
				rxmsg->len.no1[0] = data;
				status->packet_idx = 0;
	      status->parse_state = PARSE_STATE_GOT_LEN0;
			break;
		case PARSE_STATE_GOT_LEN0:
				_RTCM_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = data;
				if (status->packet_idx == rxmsg->len.no2)
				{
					status->parse_state = PARSE_STATE_GOT_PAYLOAD;
				}
			break;
		case PARSE_STATE_GOT_PAYLOAD:
			crc_init(&rxmsg->crc.no2);
			crc_update_buf((unsigned char *)&rxmsg->magic, 1, &rxmsg->crc.no2);
			crc_update_buf((unsigned char *)&rxmsg->len.no1[1], 1, &rxmsg->crc.no2);
			crc_update_buf((unsigned char *)&rxmsg->len.no1[0], 1, &rxmsg->crc.no2);
			crc_update_buf((unsigned char *)&rxmsg->payload[0], rxmsg->len.no2, &rxmsg->crc.no2);
			
			if (data != rxmsg->crc.no1[2])
			{
				status->frame_status = FRAMING_BAD_CRC;
				status->parse_state = PARSE_STATE_IDLE;
				if (data == RTCM_STX)
				{
					rxmsg->len.no2 = 0;
					rxmsg->magic = data;
					status->parse_state = PARSE_STATE_GOT_STX;
				}
			}
			else
			{
				status->parse_state = PARSE_STATE_GOT_CRC2;
			}
			break;
		case PARSE_STATE_GOT_CRC2:
			if (data != rxmsg->crc.no1[1])
			{
				status->frame_status = FRAMING_BAD_CRC;
				status->parse_state = PARSE_STATE_IDLE;
				if (data == RTCM_STX)
				{
					rxmsg->len.no2 = 0;
					rxmsg->magic = data;
					status->parse_state = PARSE_STATE_GOT_STX;
				}
			}
			else
			{
				status->parse_state = PARSE_STATE_GOT_CRC1;
			}
			break;
		case PARSE_STATE_GOT_CRC1:
			if (data != rxmsg->crc.no1[0])
			{
				status->frame_status = FRAMING_BAD_CRC;
				if (data == RTCM_STX)
				{
					rxmsg->len.no2 = 0;
					rxmsg->magic = data;
					status->parse_state = PARSE_STATE_GOT_STX;
				}
			}
			else
			{
				status->frame_status = FRAMING_OK;
				status->parse_state = PARSE_STATE_IDLE;
			}
			break;
		default:
			break;
	}
	
	if (status->frame_status == FRAMING_OK)
	{
		status->packet_rx_success_count++;
	}
	else if (status->frame_status == FRAMING_BAD_CRC)
	{
	    status->parse_error++;
	}
	return status->frame_status;
}

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */
#define PI          3.1415926535897932  /* pi */
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double dot(const double *a, const double *b, int n)
{
    volatile double c=0.0000;
    
    while (--n>=0) 
		c+=a[n]*b[n];
    return c;
}
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=dot(r,r,2),z,zk,v=RE_WGS84,sinp;
    
    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) 
	{
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}

#define LOADBITS(a) \
{ \
  while((a) > numbits) \
  { \
    if(!size--) break; \
    bitfield = (bitfield<<8)|*(data++); \
    numbits += 8; \
  } \
}

/* extract bits from data stream
   b = variable to store result, a = number of bits */
#define GETBITS64(b, a) \
{ \
  if(((a) > 56) && ((a)-56) > numbits) \
  { \
    uint64_t x; \
    GETBITS(x, 56) \
    LOADBITS((a)-56) \
    b = ((x<<((a)-56)) | (bitfield<<(sizeof(bitfield)*8-numbits)) \
    >>(sizeof(bitfield)*8-((a)-56))); \
    numbits -= ((a)-56); \
  } \
  else \
  { \
    GETBITS(b, a) \
  } \
}

/* extract bits from data stream
   b = variable to store result, a = number of bits */
#define GETBITS(b, a) \
{ \
  LOADBITS(a) \
  b = (bitfield<<(64-numbits))>>(64-(a)); \
  numbits -= (a); \
}

/* extract bits from data stream
   b = variable to store result, a = number of bits */
#define GETBITSFACTOR(b, a, c) \
{ \
  LOADBITS(a) \
  b = ((bitfield<<(sizeof(bitfield)*8-numbits))>>(sizeof(bitfield)*8-(a)))*(c); \
  numbits -= (a); \
}

/* extract floating value from data stream
   b = variable to store result, a = number of bits */
#define GETFLOAT(b, a, c) \
{ \
  LOADBITS(a) \
  b = ((double)((bitfield<<(64-numbits))>>(64-(a))))*(c); \
  numbits -= (a); \
}

/* extract signed floating value from data stream
   b = variable to store result, a = number of bits */
#define GETFLOATSIGN(b, a, c) \
{ \
  LOADBITS(a) \
  b = ((double)(((int64_t)(bitfield<<(64-numbits)))>>(64-(a))))*(c); \
  numbits -= (a); \
}

/* extract bits from data stream
   b = variable to store result, a = number of bits */
#define GETBITSSIGN(b, a) \
{ \
  LOADBITS(a) \
  b = ((int64_t)(bitfield<<(64-numbits)))>>(64-(a)); \
  numbits -= (a); \
}

#define GETFLOATSIGNM(b, a, c) \
{ int l; \
  LOADBITS(a) \
  l = (bitfield<<(64-numbits))>>(64-1); \
  b = ((double)(((bitfield<<(64-(numbits-1))))>>(64-(a-1))))*(c); \
  numbits -= (a); \
  if(l) b *= -1.0; \
}

#define SKIPBITS(b) { LOADBITS(b) numbits -= (b); }

unsigned char decode_rtcm(rtcm_message_t *msg, double *latitude, double *longitude, double *altitude)
{
	unsigned long long numbits = 0, bitfield = 0;
	unsigned int size = (msg->len.no2 & 0x3FF), type;
	unsigned char *data = msg->payload;
	long long ant_XYZ_ecef[3];
	short ant_H;
	double ecef_temp[3],wgs84[3];
	float antenna_height;
	GETBITS(type,12)
	switch(type)
	{
    	case 1005: case 1006:
			SKIPBITS(22)
			GETBITSSIGN(ant_XYZ_ecef[0], 38)
			SKIPBITS(2)
			GETBITSSIGN(ant_XYZ_ecef[1], 38)
			SKIPBITS(2)
			GETBITSSIGN(ant_XYZ_ecef[2], 38)
			if(type == 1006)
				GETBITS(ant_H, 16)
			
			ecef_temp[0] = ant_XYZ_ecef[0]*0.0001;
			ecef_temp[1] = ant_XYZ_ecef[1]*0.0001;
			ecef_temp[2] = ant_XYZ_ecef[2]*0.0001;
			antenna_height = ant_H*0.0001f;
			ecef2pos(ecef_temp, wgs84);
			
			*latitude=wgs84[0]*180.0/PI;
			*longitude=wgs84[1]*180.0/PI;
			*altitude=wgs84[2];
			return 1;
			break;
    	default:
      		break;
	}
	return 0;
}

unsigned char parse_rtcm(unsigned char * databuf, unsigned int buf_length)
{
	unsigned int index;
	static rtcm_status_t status;
	static rtcm_message_t msg;
	static double latitude, longitude,altitude;
	for(index = 0; index < buf_length; index++)
	{
		if(frame_char_buf(databuf[index], &status, &msg) == FRAMING_OK)
		{
			if(decode_rtcm(&msg, &latitude, &longitude,&altitude))
			{
				//已经获取到rtk基站位置
				printf("latitude:%lf,longitude:%lf,altitude(HAE):%lf\r\n",latitude,longitude,altitude);
				return 1;
			}
		}
	}
	return 0;
}

int main (int argc, char *argv[])
{
	int fd;
	int len, i,ret;
    const char buf[] = "Valar morghulis!\r\n";
	unsigned char temp_reveive_buf[4100];
	const unsigned char temp_buf_rtcm[40]={0xff,0xff,0xff,0xff, 0xD3, 0x00, 0x15, 0x3E, 0xE0, 0x00, 0x03, 0xFA, 0xCD, 0x2D, 0xAA, 0x60, 0x0A, 0x19, 0xE4, 0xC3, 0x0B, 0x09, 0x88, 0x46, 0xFF, 0x15, 0x00, 0x00, 0x69, 0xE7, 0x80, 0x55};
	fd = open(DEV_NAME, O_RDWR | O_NOCTTY);
	if(fd < 0)
	{
		perror(DEV_NAME);
		return -1;
	}
 
	len = write(fd, buf, sizeof(buf));
	if (len < 0)
	{
		printf("write data error.\r\n");
	}
	/*
	while(1)
	{
		len = read(fd, temp_reveive_buf, 4100);
		if(len)
		{
			if(parse_rtcm(temp_reveive_buf,len))
				break;
		}
		usleep(100*1000);
	}
	*/
	parse_rtcm((unsigned char *)temp_buf_rtcm,40);
	return(0);
}
