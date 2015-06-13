/*
 * Copyright (c) 2015 Free Electrons
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_graph.h>

#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

#include "drm/drmP.h"

static inline struct drm_encoder_chain *
to_encoder_chain(struct drm_encoder *encoder)
{
	return container_of(encoder, struct drm_encoder_chain, encoder);
}

static void
drm_encoder_chain_disable(struct drm_encoder *encoder)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (elem->funcs->disable)
			elem->funcs->disable(elem);
	}

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (elem->funcs->post_disable)
			elem->funcs->post_disable(elem);
	}
}

static void drm_encoder_chain_enable(struct drm_encoder *encoder)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	if (list_empty(&chain->elems))
		return;

	elem = list_last_entry(&chain->elems,
			       struct drm_encoder_element,
			       chain_node);

	if (elem->funcs.out_format.set)
		elem->funcs.out_format.set(elem, chain->bus_format);

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (elem->funcs->pre_enable)
			elem->funcs->pre_enable(elem);
	}

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (elem->funcs->enable)
			elem->funcs->enable(elem);
	}
}

static int drm_encoder_chain_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;
	const u32 *src_fmts, *dst_fmts;
	int nsrc_fmts, ndst_fmts;
	int i, j, ret;
	int ret;

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (!elem->funcs->atomic_check)
			continue;

		ret = elem->funcs->atomic_check(elem, crtc_state, conn_state);
		if (ret)
			return ret;
	}

	if (list_empty(&chain->elems))
		return 0;

	elem = list_last_entry(&chain->elems, struct drm_encoder_element,
			       chain_node);
	if (!elem->funcs->out_format.supported)
		return 0;

	src_fmts = elem->funcs->out_format.supported(elem, &nsrc_fmts);
	dst_fmts = conn_state->connector->display_info.bus_formats;
	ndst_fmts = conn_state->connector->display_info.num_bus_formats;

	/* Find the first match beetween source and dest formats */
	for (i = 0; i < nsrc_fmts; i++) {
		for (j = 0; j < ndst_fmts; j++) {
			if (src_fmts[i] == dst_fmts[i])
				break;
		}

		if (j < ndst_fmts)
			break;
	}

	if (i == nsrc_fmts)
		return -EINVAL;

	chain->bus_format = src_fmts[i];

	return 0;
}

static enum drm_connector_status
drm_encoder_chain_detect(struct drm_encoder *encoder,
			 struct drm_connector *connector)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	if (list_empty(&chain->elems))
		return connector_status_unknown;

	elem = list_last_entry(&chain->elems, struct drm_encoder_element,
			       chain_node);

	if (!elem->funcs->detect)
		return connector_status_unknown;

	return elem->funcs->detect(elem, connector);
}

static void drm_encoder_chain_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (elem->funcs->mode_set)
			elem->funcs->mode_set(elem, mode, adjusted_mode);
	}
}

static const struct drm_encoder_helper_funcs drm_encoder_chain_helper_funcs = {
	.detect = drm_encoder_chain_detect,
	.disable = drm_encoder_chain_disable,
	.enable = drm_encoder_chain_enable,
	.atomic_check = drm_encoder_chain_atomic_check,
	.mode_set = drm_encoder_chain_mode_set,
};

static void drm_encoder_chain_destroy(struct drm_encoder *encoder)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem, *tmp;

	drm_encoder_cleanup(encoder);
	list_for_each_entry_safe(elem, tmp, &chain->elems, chain_node) {
		if (!elem->funcs->destroy)
			continue;

		list_del(&elem->chain_node);
		elem->funcs->destroy(elem);
	}
}

static const struct drm_encoder_funcs drm_encoder_chain_funcs = {
	.destroy = drm_encoder_chain_destroy,
};

int drm_encoder_element_init(struct drm_device *dev,
			     struct drm_encoder_element *elem)
{
	drm_modeset_lock_all(dev);
	list_add_tail(&elem->node, &dev->mode_config.encoder_elem_list);
	drm_modeset_unlock_all(dev);

	return 0;
}
EXPORT_SYMBOL(drm_encoder_element_init);

void drm_encoder_element_cleanup(struct drm_encoder_element *elem)
{
	struct drm_device *dev = elem->dev;

	drm_modeset_lock_all(dev);
	list_del(&elem->node);
	drm_modeset_unlock_all(dev);

	memset(elem, 0, sizeof(*elem));
}
EXPORT_SYMBOL(drm_encoder_element_cleanup);

struct drm_encoder_chain *drm_encoder_chain_alloc(void)
{
	struct drm_encoder_chain *chain = kzalloc(sizeof(*chain), GFP_KERNEL);

	if (!chain)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&chain->elems);

	return chain;
}
EXPORT_SYMBOL(drm_encoder_chain_alloc);

void drm_encoder_chain_free(struct drm_encoder_chain *chain)
{
	kfree(chain);
}
EXPORT_SYMBOL(drm_encoder_chain_free);

int drm_encoder_chain_init(struct drm_device *dev,
			   struct drm_encoder_chain *chain,
			   u32 possible_crtcs,
			   u32 possible_clones)
{
	struct drm_encoder_element *last;
	int ret;

	if (list_empty(&chain->elems))
		return -EINVAL;

	last = list_last_entry(&chain->elems, struct drm_encoder_element,
			       chain_node);
	chain->dev = dev;

	drm_encoder_helper_add(&chain->encoder,
			       &drm_encoder_chain_helper_funcs);

	/*
	 * Here I'm assuming the userspace is interested in the last encoder
	 * element type.
	 */
	ret = drm_encoder_init(dev, &chain->encoder,
			       &drm_encoder_chain_funcs,
			       last->encoder_type);
	if (ret)
		return ret;

	chain->encoder.possible_crtcs = possible_crtcs;
	chain->encoder.possible_clones = possible_clones;

	list_add_tail(&chain->node, dev->mode_config.encoder_chain_list);

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_init);

static int
drm_encoder_chain_helper_choose_link_format(struct drm_encoder_element *elem,
					    struct drm_encoder_element *next)
{
	const u32 *src_fmts, *dst_fmts;
	int nsrc_fmts, ndst_fmts;
	int i, j, ret;

	if (!elem->funcs->out_format.supported ||
	    !next->funcs->in_format.supported)
		return 0;

	src_fmts = elem->funcs->out_format.supported(elem, &nsrc_fmts);
	dst_fmts = next->funcs->in_format.supported(next, &ndst_fmts);

	/* Find the first match beetween source and dest formats */
	for (i = 0; i < nsrc_fmts; i++) {
		for (j = 0; j < ndst_fmts; j++) {
			if (src_fmts[i] == dst_fmts[i])
				break;
		}

		if (j < ndst_fmts)
			break;
	}

	if (i == nsrc_fmts)
		return -EINVAL;

	if (elem->funcs->out_format.set) {
		ret = elem->funcs->out_format.set(elem, src_fmts[i]);
		if (ret)
			return ret;
	}

	if (next->funcs->in_format.set) {
		ret = next->funcs->in_format.set(next, src_fmts[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int
drm_encoder_chain_helper_config_links(struct drm_encoder_chain *chain)
{
	struct drm_encoder_element *elem;
	int ret;

	list_for_each_entry(elem, &chain->elems, chain_node) {
		if (list_is_last(&elem->chain_node, &chain->elems))
			break;

		ret = drm_encoder_chain_helper_choose_link_format(elem,
					list_next_entry(elem, chain_node));
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_helper_config_links);

int drm_encoder_chain_attach_element(struct drm_encoder_chain *chain,
				     struct drm_encoder_element *elem)
{
//	struct drm_encoder_element *prev = NULL;
//	int ret;

//	if (list_empty(&chain->elems))
//		prev = list_last_entry(&chain->elems,
//				       struct drm_encoder_element, node);

//	if (prev && prev->funcs->attach) {
//		ret = prev->funcs->attach(prev, elem);
//		if (ret)
//			return ret;
//	}

	list_add_tail(&elem->chain_node, &chain->elems);
	elem->chain = chain;

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_attach_element);

void drm_encoder_chain_detach_element(struct drm_encoder_element *elem)
{
	list_del(&elem->chain_node);
}
EXPORT_SYMBOL(drm_encoder_chain_detach_element);

static struct drm_encoder_element *
of_find_drm_encoder_element(struct drm_device *dev, struct device_node *np)
{
	struct drm_encoder_element *elem;

	list_for_each_entry(elem, &dev->mode_config.encoder_elem_list, node) {
		if (elem->of_node == np)
			return elem;
	}

	return NULL;
}

//static struct drm_encoder_element *
//of_find_drm_connector(struct drm_device *dev, struct device_node *np)
//{
//	struct drm_connector *conn;
//
//	list_for_each_entry(conn, &dev->mode_config.connector_list, head) {
//		if (conn->of_node == np)
//			return conn;
//	}
//
//	return NULL;
//}
//
//static struct drm_connector *of_create_drm_connector(struct drm_device *dev,
//						     struct device_node *np)
//{
//	struct drm_connector *conn;
//
//	conn = kzalloc(sizeof(*conn), GFP_KERNEL);
//
//	return conn;
//}

int of_create_drm_encoder_chains(struct drm_device *dev,
				 struct device_node *np,
				 u32 possible_crtcs,
				 u32 possible_clones)
{
	struct drm_encoder_element *elem;
	struct drm_encoder_chain *chain;
	struct device_node *endpoint_np = NULL, *subendpoint_np;
	struct of_endpoint endpoint;
	int ret;

	while (true) {
		endpoint_np = of_graph_get_next_endpoint(np, endpoint_np);
		if (!endpoint_np)
			break;

		chain = drm_encoder_chain_alloc();
		if (!chain)
			return ERR_PTR(chain);

		chain->dev = dev;

		elem = drm_find_encoder_element(dev, endpoint_np);
		if (!elem) {
			ret = ERR_PTR(-EINVAL);
			goto err;
		}

		ret = drm_encoder_chain_attach_element(chain, elem);
		if (ret)
			goto err;

		subendpoint_np = endpoint_np;
		while (true) {
			subendpoint_np = of_parse_phandle(subendpoint_np,
							  "remote-endpoint",
							  0);
			if (!subendpoint_np)
				break;

			elem = of_find_drm_encoder_element(dev, subendpoint_np);
			if (!elem)
				break;

			ret = drm_encoder_chain_attach_element(chain, elem);
			if (ret)
				goto err;
		}

		ret = drm_encoder_chain_init(dev, chain, possible_crtcs,
					     possible_clones);
		if (ret)
			goto err;
	}

	return 0;

err:
	drm_encoder_chain_free(chain);
	return ret;
}


/*
static int drm_encoder_chain_bind_element(struct device *dev)
{
	struct drm_encoder_chain *chain = dev_get_drvdata(dev);
	int ret;

	ret = component_bind_all(dev, chain);
	if (ret)
		return ret;

	return 0;
}

static void drm_encoder_chain_unbind_element(struct device *dev)
{

}

const struct component_master_ops *drm_encoder_chain_component_ops = {
	.bind = drm_encoder_chain_bind_element,
	.unbind = drm_encoder_chain_unbind_element,
};
*/

MODULE_AUTHOR("Boris Brezillon <boris.brezillon@free-electrons.com>");
MODULE_DESCRIPTION("DRM encoder chain infrastructure");
MODULE_LICENSE("GPL and additional rights");
