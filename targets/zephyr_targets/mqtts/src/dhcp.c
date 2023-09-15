#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(aws, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>

static struct net_mgmt_event_callback mgmt_cb;

static K_SEM_DEFINE(got_address, 0, 1);

static void handler(struct net_mgmt_event_callback *cb,
		    uint32_t mgmt_event,
		    struct net_if *iface)
{
	int i;
	bool notified = false;

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (iface->config.ip.ipv4->unicast[i].addr_type !=
		    NET_ADDR_DHCP) {
			continue;
		}

		if (!notified) {
			k_sem_give(&got_address);
			notified = true;
		}
		break;
	}
}

void app_dhcpv4_startup(void)
{
	LOG_INF("starting DHCPv4");

	net_mgmt_init_event_callback(&mgmt_cb, handler,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);

	net_dhcpv4_start(net_if_get_default());

	k_sem_take(&got_address, K_FOREVER);
}
