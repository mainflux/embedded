#include <string.h>
#include <stdbool.h>

#include "MQTTInterface.h"
#include "stm32f4xx_hal.h"
#include "config.h"
#include MBEDTLS_CONFIG_FILE
#include "mbedtls/platform.h"
#include "lwip.h"
#include "lwip/api.h"
#include "lwip/sockets.h"

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif
#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"

#define SERVER_PORT "8883"
#define DEBUG_LEVEL 1
#define PROG_DELAY 1000

const char mbedtls_root_certificate[] = ""; // Add root certificate.

const size_t mbedtls_root_certificate_len = sizeof(mbedtls_root_certificate);

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#define MEMORY_HEAP_SIZE 65536
uint8_t alloc_buf[MEMORY_HEAP_SIZE];
#endif

mbedtls_net_context server_fd;
const char *pers = "mbedtls";

mbedtls_entropy_context entropy;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt cacert;

static void my_debug(void *ctx, int level, const char *file, int line, const char *str)
{
	((void)level);
	mbedtls_fprintf((FILE *)ctx, "%s:%04d: %s", file, line, str);
	fflush((FILE *)ctx);
}

int net_init(Network *n)
{
	int ret;

	// Initialize mbedTLS realted variables
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
	mbedtls_memory_buffer_alloc_init(alloc_buf, sizeof(alloc_buf));
#endif

#if defined(MBEDTLS_DEBUG_C)
	mbedtls_debug_set_threshold(DEBUG_LEVEL);
#endif

	// Mbedtls_net_init(&server_fd); //MX_LWIP_Init() is called already
	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&conf);
	mbedtls_x509_crt_init(&cacert);
	mbedtls_ctr_drbg_init(&ctr_drbg);

	mbedtls_entropy_init(&entropy);
	if ((ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char *)pers,
									 strlen(pers))) != 0)
	{
		return ERR_CODE;
	}

	// Register functions
	n->mqttread = netRead;
	n->mqttwrite = netWrite;
	n->disconnect = netDisconnect;

	return 0;
}

int netConnect(Network *n, char *ip, int port)
{
	int ret;

	// SSL/TLS connection process.
	ret = mbedtls_x509_crt_parse(&cacert, (const unsigned char *)mbedtls_root_certificate,
								 mbedtls_root_certificate_len);
	if (ret < 0)
	{
		printf("mbedtls_x509_crt_parse failed.\n");
		return ERR_CODE;
	}

	ret = mbedtls_net_connect(&server_fd, server, SERVER_PORT,
							  MBEDTLS_NET_PROTO_TCP);
	if (ret < 0)
	{
		printf("mbedtls_net_connect failed.\n");
		return ERR_CODE;
	}

	ret = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT,
									  MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
	if (ret < 0)
	{
		printf("mbedtls_ssl_config_defaults failed.\n");
		return ERR_CODE;
	}

	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
	mbedtls_ssl_conf_ca_chain(&conf, &cacert, NULL);
	mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);
	mbedtls_ssl_conf_dbg(&conf, my_debug, stdout);

	ret = mbedtls_ssl_setup(&ssl, &conf);
	if (ret < 0)
	{
		printf("mbedtls_ssl_setup failed.\n");
		return ERR_CODE;
	}

	ret = mbedtls_ssl_set_hostname(&ssl, SERVER_NAME);
	if (ret < 0)
	{
		printf("mbedtls_ssl_set_hostname failed.\n");
		return ERR_CODE;
	}

	mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv,
						NULL);

	while ((ret = mbedtls_ssl_handshake(&ssl)) != 0)
	{
		if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
		{
			printf("mbedtls_ssl_handshake failed.\n");
			return ERR_CODE;
		}
	}

	ret = mbedtls_ssl_get_verify_result(&ssl);
	if (ret < 0)
	{
		printf("mbedtls_ssl_get_verify_result failed.\n");
		return ERR_CODE;
	}

	return 0;
}

int netRead(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
	int ret;
	int received = 0;
	bool error = false;
	bool complete = false;

	// Set timeout
	if (timeout_ms != 0)
	{
		mbedtls_ssl_conf_read_timeout(&conf, timeout_ms);
	}

	do
	{
		ret = mbedtls_ssl_read(&ssl, buffer, len);
		if (ret > 0)
		{
			received += ret;
		}
		else if (ret != MBEDTLS_ERR_SSL_WANT_READ)
		{
			error = true;
		}
		if (received >= len)
		{
			complete = true;
		}
	} while (!error && !complete);

	return received;
}

int netWrite(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
	int ret;
	int written;

	// Check all bytes are written
	for (written = 0; written < len; written += ret)
	{
		while ((ret = mbedtls_ssl_write(&ssl, buffer + written, len - written)) <= 0)
		{
			if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
			{
				return ret;
			}
		}
	}

	return written;
}

void netDisconnect(Network *n)
{
	int ret;

	do
	{
		ret = mbedtls_ssl_close_notify(&ssl);
	} while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);

	mbedtls_ssl_session_reset(&ssl);
	mbedtls_net_free(&server_fd);
}

void netClear()
{
	mbedtls_net_free(&server_fd);
	mbedtls_x509_crt_free(&cacert);
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&conf);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
	mbedtls_memory_buffer_alloc_free();
#endif
}

uint32_t MilliTimer;

char timerIsExpired(Timer *timer)
{
	long left = timer->end_time - MilliTimer;
	return (left < 0);
}

void timerCountdownMS(Timer *timer, unsigned int timeout)
{
	timer->end_time = MilliTimer + timeout;
}

void timerCountdown(Timer *timer, unsigned int timeout)
{
	timer->end_time = MilliTimer + (timeout * PROG_DELAY);
}

int timerLeftMS(Timer *timer)
{
	long left = timer->end_time - MilliTimer;
	return (left < 0) ? 0 : left;
}

void timerInit(Timer *timer)
{
	timer->end_time = 0;
}
