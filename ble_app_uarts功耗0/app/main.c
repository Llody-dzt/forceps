//���õ�C��ͷ�ļ�
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//APP��ʱ����Ҫ���õ�ͷ�ļ�
#include "app_timer.h"

#include "bsp_btn_ble.h"
//�㲥��Ҫ���õ�ͷ�ļ�
#include "ble_advdata.h"
#include "ble_advertising.h"
//��Դ������Ҫ���õ�ͷ�ļ�
#include "nrf_pwr_mgmt.h"
//SoftDevice handler configuration��Ҫ���õ�ͷ�ļ�
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
//����д��ģ����Ҫ���õ�ͷ�ļ�
#include "nrf_ble_qwr.h"
//GATT��Ҫ���õ�ͷ�ļ�
#include "nrf_ble_gatt.h"
//���Ӳ���Э����Ҫ���õ�ͷ�ļ�
#include "ble_conn_params.h"
//����͸����Ҫ���õ�ͷ�ļ�
#include "my_ble_uarts.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "app_uart.h"

#include "nrf_drv_twi.h"
#include "nrf_mpu9250_twi_drv.h"
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "nrf_drv_saadc.h"
#include "nrf_power.h"

#define DEVICE_NAME                     "AnNeng-80"                      // �豸�����ַ��� 
#define UARTS_SERVICE_UUID_TYPE         BLE_UUID_TYPE_VENDOR_BEGIN         // ����͸������UUID���ͣ������Զ���UUID
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)   // ��С���Ӽ�� (0.1 ��) 
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)   // ������Ӽ�� (0.2 ��) 
#define SLAVE_LATENCY                   0                                  // �ӻ��ӳ� 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)    // �ල��ʱ(4 ��) 
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)              // �����״ε���sd_ble_gap_conn_param_update()�����������Ӳ����ӳ�ʱ�䣨5�룩
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)             // ����ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ����ļ��ʱ�䣨30�룩
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                  // ����������Ӳ���Э��ǰ�������Ӳ���Э�̵���������3�Σ�

#define APP_ADV_INTERVAL                320                               // �㲥��� (200ms)����λ0.625 ms 
#define APP_ADV_DURATION               	12000                                 // �㲥����ʱ�䣬��λ��10ms������Ϊ0��ʾ����ʱ 

#define APP_BLE_OBSERVER_PRIO           3               //Ӧ�ó���BLE�¼����������ȼ���Ӧ�ó������޸ĸ���ֵ
#define APP_BLE_CONN_CFG_TAG            1               //SoftDevice BLE���ñ�־

#define UART_TX_BUF_SIZE 256       //���ڷ��ͻ����С���ֽ�����
#define UART_RX_BUF_SIZE 256       //���ڽ��ջ����С���ֽ�����

//����stack dump�Ĵ�����룬��������ջ����ʱȷ����ջλ��
#define DEAD_BEEF                       0xDEADBEEF     
               
BLE_UARTS_DEF(m_uarts);                 //��������Ϊm_uarts�Ĵ���͸������ʵ��
NRF_BLE_GATT_DEF(m_gatt);               //��������Ϊm_gatt��GATTģ��ʵ��
NRF_BLE_QWR_DEF(m_qwr);                 //����һ������Ϊm_qwr���Ŷ�д��ʵ��
BLE_ADVERTISING_DEF(m_advertising);     //��������Ϊm_advertising�Ĺ㲥ģ��ʵ��



bool MPU_OPEN=false;
bool conne_flag=false;
bool wake_sta;
//�ñ������ڱ������Ӿ������ʼֵ����Ϊ������
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; 

static uint16_t   m_ble_uarts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

void configIO(void);
//const char device_name[17]={0xE5,0xAE,0x89,0xE8,0x83,0xBD,0xE5,0x8C,0xBB,0xE7,0x96,0x97,0xE4,0xB8,0x80,0x0A};
const char device_name[12]={0xE5,0xB0,0x8F,0xE7,0xAA,0x9D,0xE7,0xA7,0x91,0xE6,0x8A,0x80};


//���崮��͸������UUID�б�
static ble_uuid_t m_adv_uuids[]          =                                          
{
    {BLE_UUID_UARTS_SERVICE, UARTS_SERVICE_UUID_TYPE}
};

//GAP������ʼ�����ú���������Ҫ��GAP�����������豸���ƣ������������ѡ���Ӳ���
static void gap_params_init(void)
{
    ret_code_t              err_code;
	  //�������Ӳ����ṹ�����
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    //����GAP�İ�ȫģʽ
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    //����GAP�豸����
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)device_name,
                                          strlen(device_name));
    //��麯�����صĴ������
		APP_ERROR_CHECK(err_code);
																				
    //������ѡ���Ӳ���������ǰ������gap_conn_params
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;//��С���Ӽ��
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;//��С���Ӽ��
    gap_conn_params.slave_latency     = SLAVE_LATENCY;    //�ӻ��ӳ�
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; //�ල��ʱ
    //����Э��ջAPI sd_ble_gap_ppcp_set����GAP����
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
}
//GATT�¼����������ú����д���MTU�����¼�
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    //�����MTU�����¼�
	  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        //���ô���͸���������Ч���ݳ��ȣ�MTU-opcode-handle��
			  m_ble_uarts_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_uarts_max_data_len, m_ble_uarts_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}
//��ʼ��GATT����ģ��
static void gatt_init(void)
{
    //��ʼ��GATT����ģ��
	  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	  //��麯�����صĴ������
    APP_ERROR_CHECK(err_code);
	  //����ATT MTU�Ĵ�С,�������õ�ֵΪ247
	  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

//�Ŷ�д���¼������������ڴ����Ŷ�д��ģ��Ĵ���
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    //���������
	  APP_ERROR_HANDLER(nrf_error);
}

//����͸���¼��ص�����������͸�������ʼ��ʱע��
static void uarts_data_handler(ble_uarts_evt_t * p_evt)
{
	//�ж��¼�����:���յ��������¼�
    if (p_evt->type == BLE_UARTS_EVT_RX_DATA)
    {
        uint32_t err_code;
        //���ڴ�ӡ�����յ�����
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }				
    }
		
		//�ж��¼�����:���;����¼������¼��ں����������õ�����ǰ�����ڸ��¼��з�תָʾ��D4��״̬��ָʾ���¼��Ĳ���
//    if (p_evt->type == BLE_UARTS_EVT_TX_RDY)
//    {
//	//		nrf_gpio_pin_toggle(LED_4);
//		}
}
//�����ʼ����������ʼ���Ŷ�д��ģ��ͳ�ʼ��Ӧ�ó���ʹ�õķ���
static void services_init(void)
{
    ret_code_t         err_code;
	  //���崮��͸����ʼ���ṹ��
	  ble_uarts_init_t     uarts_init;
	  //�����Ŷ�д���ʼ���ṹ�����
    nrf_ble_qwr_init_t qwr_init = {0};

    //�Ŷ�д���¼�������
    qwr_init.error_handler = nrf_qwr_error_handler;
    //��ʼ���Ŷ�д��ģ��
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
		//��麯������ֵ
    APP_ERROR_CHECK(err_code);
    
		
		/*------------------���´����ʼ������͸������-------------*/
		//���㴮��͸�������ʼ���ṹ��
		memset(&uarts_init, 0, sizeof(uarts_init));
		//���ô���͸���¼��ص�����
    uarts_init.data_handler = uarts_data_handler;
    //��ʼ������͸������
    err_code = ble_uarts_init(&m_uarts, &uarts_init);
    APP_ERROR_CHECK(err_code);
		/*------------------��ʼ������͸������-END-----------------*/
}

//���Ӳ���Э��ģ���¼�������
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    //�ж��¼����ͣ������¼�����ִ�ж���
	  //���Ӳ���Э��ʧ��
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
		//���Ӳ���Э�̳ɹ�
		if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
       //���ܴ���;
    }
}

//���Ӳ���Э��ģ��������¼�������nrf_error�����˴�����룬ͨ��nrf_error���Է���������Ϣ
static void conn_params_error_handler(uint32_t nrf_error)
{
    //���������
	  APP_ERROR_HANDLER(nrf_error);
}


//���Ӳ���Э��ģ���ʼ��
static void conn_params_init(void)
{
    ret_code_t             err_code;
	  //�������Ӳ���Э��ģ���ʼ���ṹ��
    ble_conn_params_init_t cp_init;
    //����֮ǰ������
    memset(&cp_init, 0, sizeof(cp_init));
    //����ΪNULL����������ȡ���Ӳ���
    cp_init.p_conn_params                  = NULL;
	  //���ӻ�����֪ͨ���״η������Ӳ�����������֮���ʱ������Ϊ5��
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	  //ÿ�ε���sd_ble_gap_conn_param_update()�����������Ӳ������������֮��ļ��ʱ������Ϊ��30��
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	  //�������Ӳ���Э��ǰ�������Ӳ���Э�̵�����������Ϊ��3��
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	  //���Ӳ������´������¼���ʼ��ʱ
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	  //���Ӳ�������ʧ�ܲ��Ͽ�����
    cp_init.disconnect_on_fail             = false;
	  //ע�����Ӳ��������¼����
    cp_init.evt_handler                    = on_conn_params_evt;
	  //ע�����Ӳ������´����¼����
    cp_init.error_handler                  = conn_params_error_handler;
    //���ÿ⺯���������Ӳ������³�ʼ���ṹ��Ϊ�����������ʼ�����Ӳ���Э��ģ��
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

//�㲥�¼�������
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
    //�жϹ㲥�¼�����
    switch (ble_adv_evt)
    {
        //���ٹ㲥�����¼������ٹ㲥�������������¼�
			  case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
			      //���ù㲥ָʾ��Ϊ���ڹ㲥��D1ָʾ����˸��
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        //�㲥IDLE�¼����㲥��ʱ���������¼�
        case BLE_ADV_EVT_IDLE:
						twi_master_init();
						u8	res = 0x80;
						nrf_drv_mpu_write_registersd(0x68, 0x6B, 1, &res);
						res = 0x40;
						nrf_drv_mpu_write_registersd(0x68, 0x6B, 1, &res);
						wake_sta=nrf_gpio_pin_read(30);
	
					if(wake_sta)
					{
						nrf_gpio_cfg_sense_input(30,NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
					}
					else{
						nrf_gpio_cfg_sense_input(30,NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
					}	
					nrf_power_system_off();
         break;

        default:
            break;
    }
}
//�㲥��ʼ��
static void advertising_init(void)
{
    ret_code_t             err_code;
	  //����㲥��ʼ�����ýṹ�����
    ble_advertising_init_t init;
    //����֮ǰ������
    memset(&init, 0, sizeof(init));
    //�豸�������ͣ�ȫ��
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	  //�Ƿ������ۣ�����
    init.advdata.include_appearance      = false;
	  //Flag:һ��ɷ���ģʽ����֧��BR/EDR
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	  //UUID�ŵ�ɨ����Ӧ����
	  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	
    //���ù㲥ģʽΪ���ٹ㲥
    init.config.ble_adv_fast_enabled  = true;
	  //���ù㲥����͹㲥����ʱ��
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    //�㲥�¼��ص�����
    init.evt_handler = on_adv_evt;
    //��ʼ���㲥
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    //���ù㲥���ñ�ǡ�APP_BLE_CONN_CFG_TAG�����ڸ��ٹ㲥���õı�ǣ�����Ϊδ��Ԥ����һ���������ڽ�����SoftDevice�汾�У�
		//����ʹ��sd_ble_gap_adv_set_configure()�����µĹ㲥����
		//��ǰSoftDevice�汾��S132 V7.2.0�汾��֧�ֵ����㲥������Ϊ1�����APP_BLE_CONN_CFG_TAGֻ��д1��
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

//BLE�¼�������
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
		u8 res;
    //�ж�BLE�¼����ͣ������¼�����ִ����Ӧ����
    switch (p_ble_evt->header.evt_id)
    {
        //�Ͽ������¼�
			  case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						MPU_OPEN=false;
						conne_flag=false;
				
						twi_master_init();
						res = 0x80;
						nrf_drv_mpu_write_registersd(0x68, 0x6B, 1, &res);
						res = 0x40;
						nrf_drv_mpu_write_registersd(0x68, 0x6B, 1, &res);
				
//				    printf("Disconnected.");
            break;
				
        //�����¼�
        case BLE_GAP_EVT_CONNECTED:
//            printf("Connected.");
									
				    //����ָʾ��״̬Ϊ����״̬����ָʾ��D1����
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
				    //�������Ӿ��
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    //�����Ӿ��������Ŷ�д��ʵ����������Ŷ�д��ʵ���͸����ӹ��������������ж�����ӵ�ʱ��ͨ��������ͬ���Ŷ�д��ʵ�����ܷ��㵥�������������
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);			

						conne_flag=true;			
						mpu_dmp_init();				
            break;
				
        //PHY�����¼�
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
 //           printf("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
						//��ӦPHY���¹��
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
				//ϵͳ���Է������ڵȴ���
				case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            //ϵͳ����û�д洢������ϵͳ����
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        //GATT�ͻ��˳�ʱ�¼�
        case BLE_GATTC_EVT_TIMEOUT:
//            printf("GATT Client Timeout.");
				    //�Ͽ���ǰ����
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
				
        //GATT��������ʱ�¼�
        case BLE_GATTS_EVT_TIMEOUT:
//            printf("GATT Server Timeout.");
				    //�Ͽ���ǰ����
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

//��ʼ��BLEЭ��ջ
static void ble_stack_init(void)
{
    ret_code_t err_code;
    //����ʹ��SoftDevice���ú����л����sdk_config.h�ļ��е�Ƶʱ�ӵ����������õ�Ƶʱ��
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    //���屣��Ӧ�ó���RAM��ʼ��ַ�ı���
    uint32_t ram_start = 0;
	  //ʹ��sdk_config.h�ļ���Ĭ�ϲ�������Э��ջ����ȡӦ�ó���RAM��ʼ��ַ�����浽����ram_start
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    //ʹ��BLEЭ��ջ
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    //ע��BLE�¼��ص�����
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
//��ʼ����Դ����ģ��
static void power_management_init(void)
{
    ret_code_t err_code;
	  //��ʼ����Դ����
    err_code = nrf_pwr_mgmt_init();
	  //��麯�����صĴ������
    APP_ERROR_CHECK(err_code);
}

//��ʼ��ָʾ��
static void leds_init(void)
{
    ret_code_t err_code;
    //��ʼ��BSPָʾ��
    err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);

}
//��ʼ��APP��ʱ��ģ��
static void timers_init(void)
{
    //��ʼ��APP��ʱ��ģ��
    ret_code_t err_code = app_timer_init();
	  //��鷵��ֵ
    APP_ERROR_CHECK(err_code);

    //���봴���û���ʱ����Ĵ��룬�����û���ʱ���� 

}
static void log_init(void)
{
    //��ʼ��log����ģ��
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //����log����նˣ�����sdk_config.h�е�������������ն�ΪUART����RTT��
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//����״̬�����������û�й������־��������˯��ֱ����һ���¼���������ϵͳ
static void idle_state_handle(void)
{
    //��������log
	  if (NRF_LOG_PROCESS() == false)
    {
        //���е�Դ�����ú�����Ҫ�ŵ���ѭ������ִ��
			  nrf_pwr_mgmt_run();
    }
}
//�����㲥���ú������õ�ģʽ����͹㲥��ʼ�������õĹ㲥ģʽһ��
static void advertising_start(void)
{
   //ʹ�ù㲥��ʼ�������õĹ㲥ģʽ�����㲥
	 ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	 //��麯�����صĴ������
   APP_ERROR_CHECK(err_code);
}

//�����¼��ص����������ڳ�ʼ��ʱע�ᣬ�ú������ж��¼����Ͳ����д���
//�����յ����ݳ��ȴﵽ�趨�����ֵ���߽��յ����з�������Ϊһ�����ݽ�����ɣ�֮�󽫽��յ����ݷ��͸�����
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_UARTS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
    //�ж��¼�����
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY://���ڽ����¼�
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            //���մ������ݣ������յ����ݳ��ȴﵽm_ble_uarts_max_data_len���߽��յ����з�����Ϊһ�����ݽ������
            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_uarts_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                    //���ڽ��յ�����ʹ��notify���͸�BLE����
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_uarts_data_send(&m_uarts, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;
        //ͨѶ�����¼������������
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        //FIFO�����¼������������
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

//��������
void uart_config(void)
{
	uint32_t err_code;
	
	//���崮��ͨѶ�������ýṹ�岢��ʼ��
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//����uart��������
    TX_PIN_NUMBER,//����uart��������
    RTS_PIN_NUMBER,//����uart RTS���ţ����عرպ���Ȼ������RTS��CTS���ţ����������������ԣ������������������ţ����������Կ���ΪIOʹ��
    CTS_PIN_NUMBER,//����uart CTS����
    APP_UART_FLOW_CONTROL_DISABLED,//�ر�uartӲ������
    false,//��ֹ��ż����
    NRF_UART_BAUDRATE_115200//uart����������Ϊ115200bps
  };
  //��ʼ�����ڣ�ע�ᴮ���¼��ص�����
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
	
}

extern float quater[4];
#define SAMPLES_BUFFER_LEN 5
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_BUFFER_LEN];
static uint32_t       m_adc_evt_counter;
float voltage;

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{  
	  if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        //���úû��棬Ϊ��һ�β���׼��
		    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_BUFFER_LEN);
        APP_ERROR_CHECK(err_code);
        int i;
        for (i = 0; i < SAMPLES_BUFFER_LEN; i++)
        {
			      //�����������ֵ����õ��ĵ�ѹֵ����ѹֵ = ����ֵ * 3.6 /2^10
			      voltage = p_event->data.done.p_buffer[i] ;//* 3.6 /1024;	
        }
        //�¼�������1
		    m_adc_evt_counter++;
    }
}
void saadc_init(void)
{
    ret_code_t err_code;
	//����ADCͨ�����ýṹ�壬��ʹ�õ��˲������ú��ʼ��
	//��ΪҪ��������оƬ��VDD�����Ե��˲������ú��ͨ������Ϊ��NRF_SAADC_INPUT_VDD
    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    //��ʼ��SAADC��ע���¼��ص�������
	  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);
    //��ʼ��SAADCͨ��0
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
	
	//���û���1��������1��ַ��ֵ��SAADC���������еĿ��ƿ�m_cb��һ������ָ��
		err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_BUFFER_LEN);
    APP_ERROR_CHECK(err_code);
    //���û���2��������1��ַ��ֵ��SAADC���������еĿ��ƿ�m_cb�Ķ�������ָ��
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_BUFFER_LEN);
    APP_ERROR_CHECK(err_code);
}

void configIO()
{
	nrf_gpio_cfg_output(MPU9250_AD0_PIN);
	mpu9250_ad0_config(false);
	nrf_gpio_cfg_output(MPU9250_CS_PIN);
	nrf_gpio_pin_set(MPU9250_CS_PIN);
	nrf_gpio_cfg_input(30,NRF_GPIO_PIN_PULLUP);
	saadc_init();
}
int main(void)
{
	float pitch,roll,yaw; 
	char data_buf[BLE_UARTS_MAX_DATA_LEN];
	uint8_t dataM_array[BLE_UARTS_MAX_DATA_LEN];
	uint16_t length;
	int IO_state;	
	
	log_init();
	uart_config();
	timers_init();
//	leds_init();
	power_management_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();	
  conn_params_init();
		
//  printf("BLE Template example started.");  

	static ble_gap_addr_t my_addr;	
	my_addr.addr[0] = 0x11;
	my_addr.addr[1] = 0xdd;
	my_addr.addr[2] = 0x08;
	my_addr.addr[3] = 0x44;
	my_addr.addr[4] = 0x44;
	my_addr.addr[5] = 0xcc;	
	my_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;			
	sd_ble_gap_addr_set(&my_addr);
	
	
	advertising_start();
	
	nrf_gpio_cfg_output(MPU9250_AD0_PIN);
	mpu9250_ad0_config(false);
	nrf_gpio_cfg_output(MPU9250_CS_PIN);
	nrf_gpio_pin_set(MPU9250_CS_PIN);
	nrf_gpio_cfg_input(30,NRF_GPIO_PIN_PULLUP);
	saadc_init();
	
	while(true){	
			if(conne_flag){		
				if(MPU_OPEN){
						if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0){
							IO_state=nrf_gpio_pin_read(30);
							nrfx_saadc_sample();	
							sprintf(data_buf,"%f,%f,%f,%f,%f,%d",quater[0],quater[1],quater[2],quater[3],voltage,IO_state);
							length=sizeof(data_buf);
							memcpy(dataM_array,data_buf,length);
							ble_uarts_data_send(&m_uarts, dataM_array, &length, m_conn_handle);
						}
				}
			}			
//			idle_state_handle();
	}
}
