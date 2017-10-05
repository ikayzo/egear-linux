#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include "dsa_priv.h"

netdev_tx_t vid_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	u16 vlan_tci = 1 << p->port;

	if (skb_vlan_push(skb, htons(ETH_P_8021Q), vlan_tci) != 0)
		goto out_free;

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	skb->dev = p->parent->dst->master_netdev;
	dev_queue_xmit(skb);

	return NETDEV_TX_OK;

out_free:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int vid_rcv(struct sk_buff *skb, struct net_device *dev,
		struct packet_type *pt, struct net_device *orig_dev)
{
	struct dsa_switch *ds = dev->dsa_ptr->ds[0];
	u8 source_port = 0;

	skb_vlan_untag(skb);

	source_port = ffs(skb->vlan_tci) - 1;
	if (source_port >= DSA_MAX_PORTS || ds->ports[source_port] == NULL)
		goto out_drop;

	skb->vlan_tci = 0;
	skb->pkt_type = PACKET_HOST;
	skb->dev = ds->ports[source_port];

	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
	return 0;
}

const struct dsa_device_ops vid_netdev_ops = {
	.xmit	= vid_xmit,
	.rcv	= vid_rcv,
};

