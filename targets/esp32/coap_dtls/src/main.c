#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "protocol_examples_common.h"
#include "coap3/coap.h"
#include "config.h"
#include "cnetwork.h"

const static char *TAG = CLIENTID;

static int resp_wait = 1;
static coap_optlist_t *optlist = NULL;
static int wait_ms;


void format_mainflux_message_topic(void)
{
    const char *_preId = "channels/";
    const char *_postId = "/messages";
    strcpy(mfTopic, _preId);
    strcat(mfTopic, mfChannelId);
    strcat(mfTopic, _postId);
}

static coap_session_t *
coap_start_pki_session(coap_context_t *ctx, coap_address_t *dst_addr, coap_uri_t *uri, coap_proto_t proto)
{
    unsigned int ca_pem_bytes = ca_pem_end - ca_pem_start;
    unsigned int client_crt_bytes = client_crt_end - client_crt_start;
    unsigned int client_key_bytes = client_key_end - client_key_start;
    static coap_dtls_pki_t dtls_pki;
    static char client_sni[256];

    memset (&dtls_pki, 0, sizeof(dtls_pki));
    dtls_pki.version = COAP_DTLS_PKI_SETUP_VERSION;
    if (ca_pem_bytes) {
        dtls_pki.verify_peer_cert        = 1;
        dtls_pki.check_common_ca         = 1;
        dtls_pki.allow_self_signed       = 1;
        dtls_pki.allow_expired_certs     = 1;
        dtls_pki.cert_chain_validation   = 1;
        dtls_pki.cert_chain_verify_depth = 2;
        dtls_pki.check_cert_revocation   = 1;
        dtls_pki.allow_no_crl            = 1;
        dtls_pki.allow_expired_crl       = 1;
        dtls_pki.allow_bad_md_hash       = 1;
        dtls_pki.allow_short_rsa_length  = 1;
        dtls_pki.validate_cn_call_back   = verify_cn_callback;
        dtls_pki.cn_call_back_arg        = NULL;
        dtls_pki.validate_sni_call_back  = NULL;
        dtls_pki.sni_call_back_arg       = NULL;
        memset(client_sni, 0, sizeof(client_sni));
        if (uri->host.length) {
            memcpy(client_sni, uri->host.s, MIN(uri->host.length, sizeof(client_sni)));
        } else {
            memcpy(client_sni, "localhost", 9);
        }
        dtls_pki.client_sni = client_sni;
    }
    dtls_pki.pki_key.key_type = COAP_PKI_KEY_PEM_BUF;
    dtls_pki.pki_key.key.pem_buf.public_cert = client_crt_start;
    dtls_pki.pki_key.key.pem_buf.public_cert_len = client_crt_bytes;
    dtls_pki.pki_key.key.pem_buf.private_key = client_key_start;
    dtls_pki.pki_key.key.pem_buf.private_key_len = client_key_bytes;
    dtls_pki.pki_key.key.pem_buf.ca_cert = ca_pem_start;
    dtls_pki.pki_key.key.pem_buf.ca_cert_len = ca_pem_bytes;

    return coap_new_client_session_pki(ctx, NULL, dst_addr, proto,
                                       &dtls_pki);
}

static coap_response_t message_handler(coap_session_t *session,
                                       const coap_pdu_t *sent,
                                       const coap_pdu_t *received,
                                       const coap_mid_t mid)
{
    const unsigned char *data = NULL;
    size_t data_len;
    size_t offset;
    size_t total;
    coap_pdu_code_t rcvd_code = coap_pdu_get_code(received);

    if (COAP_RESPONSE_CLASS(rcvd_code) == 2)
    {
        if (coap_get_data_large(received, &data_len, &data, &offset, &total))
        {
            if (data_len != total)
            {
                printf("Unexpected partial data received offset %u, length %u\n", offset, data_len);
            }
            printf("Received:\n%.*s\n", (int)data_len, data);
            resp_wait = 0;
        }
        return COAP_RESPONSE_OK;
    }
    printf("%d.%02d", (rcvd_code >> 5), rcvd_code & 0x1F);
    if (coap_get_data_large(received, &data_len, &data, &offset, &total))
    {
        printf(": ");
        while (data_len--)
        {
            printf("%c", isprint(*data) ? *data : '.');
            data++;
        }
    }
    printf("\n");
    resp_wait = 0;
    return COAP_RESPONSE_OK;
}

static void coap_log_handler(coap_log_t level, const char *message)
{
    uint32_t esp_level = ESP_LOG_INFO;
    const char *cp = strchr(message, '\n');

    while (cp)
    {
        ESP_LOG_LEVEL(esp_level, TAG, "%.*s", (int)(cp - message), message);
        message = cp + 1;
        cp = strchr(message, '\n');
    }
    if (message[0] != '\000')
    {
        ESP_LOG_LEVEL(esp_level, TAG, "%s", message);
    }
}

static void coap_client(void *p)
{
    coap_address_t uri;
    static coap_uri_t dst_addr;
    const char *server_uri = server;
    coap_context_t *ctx = NULL;
    coap_session_t *session = NULL;
    coap_pdu_t *request = NULL;
    unsigned char token[8];
    size_t tokenlength;
    coap_addr_info_t *info_list = NULL;
    coap_proto_t proto;
    char tmpbuf[INET6_ADDRSTRLEN];

    coap_startup();

    coap_set_log_handler(coap_log_handler);
    coap_set_log_level(COAP_LOG_DEFAULT_LEVEL);

    ctx = coap_new_context(NULL);
    if (!ctx)
    {
        ESP_LOGE(TAG, "coap_new_context() failed");
        goto clean_up;
    }
    coap_context_set_block_mode(ctx,
                                COAP_BLOCK_USE_LIBCOAP | COAP_BLOCK_SINGLE_BODY);

    coap_register_response_handler(ctx, message_handler);

    if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1)
    {
        ESP_LOGE(TAG, "CoAP server uri %s error", server_uri);
        goto clean_up;
    }

    info_list = coap_resolve_address_info(&uri.host, uri.port, uri.port,
                                          uri.port, uri.port,
                                          0,
                                          1 << uri.scheme,
                                          COAP_RESOLVE_TYPE_REMOTE);

    if (info_list == NULL)
    {
        ESP_LOGE(TAG, "failed to resolve address");
        clean_up();
    }
    proto = info_list->proto;
    memcpy(&dst_addr, &info_list->addr, sizeof(dst_addr));
    coap_free_address_info(info_list);
    session = coap_start_pki_session(ctx, &dst_addr, &uri, proto);
    if (!session)
    {
        ESP_LOGE(TAG, "coap_new_client_session() failed");
        clean_up();
    }

    while (true)
    {
        request = coap_new_pdu(coap_is_mcast(&dst_addr) ? COAP_MESSAGE_NON : COAP_MESSAGE_CON,
                               COAP_REQUEST_CODE_GET, session);
        if (!request)
        {
            ESP_LOGE(TAG, "coap_new_pdu() failed");
            goto clean_up;
        }
        coap_session_new_token(session, &tokenlength, token);
        coap_add_token(request, tokenlength, token);
        coap_add_optlist_pdu(request, &optlist);

        resp_wait = 1;
        coap_send(session, request);

        wait_ms = COAP_DEFAULT_TIME_SEC * MS_COUNT;

        while (resp_wait)
        {
            int result = coap_io_process(ctx, wait_ms > 1000 ? 1000 : wait_ms);
            if (result >= 0)
            {
                if (result >= wait_ms)
                {
                    ESP_LOGE(TAG, "No response from server");
                    break;
                }
                else
                {
                    wait_ms -= result;
                }
            }
        }
        for (int countdown = 10; countdown >= 0; countdown--)
        {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}
void clean_up()
{
    if (session)
    {
        coap_session_release(session);
    }
    if (ctx)
    {
        coap_free_context(ctx);
    }
    coap_cleanup();

    ESP_LOGI(TAG, "Finished");
    vTaskDelete(NULL);
}
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(wifi_init_softap());

    xTaskCreate(coap_client, "coap", COAP_BUFFSIZE, NULL, COAP_TIMEOUT, NULL);
}