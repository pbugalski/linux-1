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

	list_for_each_entry(elem, &chain->elems, node) {
		if (elem->funcs->disable)
			elem->funcs->disable(elem);
	}

	list_for_each_entry(elem, &chain->elems, node) {
		if (elem->funcs->post_disable)
			elem->funcs->post_disable(elem);
	}
}

static void drm_encoder_chain_enable(struct drm_encoder *encoder)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	list_for_each_entry(elem, &chain->elems, node) {
		if (elem->funcs->pre_enable)
			elem->funcs->pre_enable(elem);
	}

	list_for_each_entry(elem, &chain->elems, node) {
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
	int ret;

	list_for_each_entry(elem, &chain->elems, node) {
		if (!elem->funcs->atomic_check)
			continue;

		ret = elem->funcs->atomic_check(elem, crtc_state, conn_state);
		if (ret)
			return ret;
	}

	return 0;
}

static enum drm_connector_status
drm_encoder_chain_detect(struct drm_encoder *encoder,
			 struct drm_connector *connector)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	enum drm_connector_status ret = connector_status_unknown;
	struct drm_encoder_element *elem;

	if (list_empty(&chain->elems))
		return ret;

	elem = list_last_entry(&chain->elems, struct drm_encoder_element,
			       node);

	if (!elem->funcs->detect)
		return ret;

	return elem->funcs->detect(elem, connector);
}

static void drm_encoder_chain_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_chain *chain = to_encoder_chain(encoder);
	struct drm_encoder_element *elem;

	list_for_each_entry(elem, &chain->elems, node) {
		if (elem->funcs->mode_set)
			elem->funcs->mode_set(elem, mode, adjusted_mode);
	}
}

static const struct drm_encoder_helper_funcs drm_encoder_chain_helper_funcs = {
//	struct drm_crtc *(*get_crtc)(struct drm_encoder *encoder);
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
	list_for_each_entry_safe(elem, tmp, &chain->elems, node) {
		if (!elem->funcs->destroy)
			continue;

		list_del(&elem->node);
		elem->funcs->destroy(elem);
	}
}

static const struct drm_encoder_funcs drm_encoder_chain_funcs = {
	.destroy = drm_encoder_chain_destroy,
};

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
	struct drm_encoder_element *first, *last;
	int ret;

	if (list_empty(&chain->elems))
		return -EINVAL;

	first = list_first_entry(&chain->elems, struct drm_encoder_element,
				 node);
	last = list_last_entry(&chain->elems, struct drm_encoder_element,
			       node);
	chain->dev = dev;

	drm_encoder_helper_add(&chain->encoder,
			       &drm_encoder_chain_helper_funcs);

	/*
	 * FIXME: which encoder_type should we expose ?
	 * Here I'm assuming the userspace is interested in the last encoder
	 * element type.
	 */
	ret = drm_encoder_init(dev, &chain->encoder,
			       &drm_encoder_chain_funcs,
			       last->encoder_type);
	if (ret)
		return ret;

	chain->encoder.possible_crtcs = first->possible_crtcs;
	chain->encoder.possible_clones = first->possible_clones;

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_init);

int drm_encoder_chain_helper_attach(struct drm_encoder_element *elem,
				    struct drm_encoder_element *next)
{
	const u32 *src_fmts, *dst_fmts;
	int nsrc_fmts, ndst_fmts;
	int i, j, ret;

	if (!elem->funcs->supported_bus_formats ||
	    !next->funcs->supported_bus_formats)
		return 0;

	src_fmts = elem->funcs->supported_bus_formats(elem, &nsrc_fmts);
	dst_fmts = next->funcs->supported_bus_formats(next, &ndst_fmts);

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

	if (elem->funcs->set_bus_format) {
		ret = elem->funcs->set_bus_format(elem, src_fmts[i]);
		if (ret)
			return ret;
	}

	if (elem->funcs->set_bus_format) {
		ret = next->funcs->set_bus_format(next, src_fmts[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_helper_attach);

int drm_encoder_chain_attach_element(struct drm_encoder_chain *chain,
				     struct drm_encoder_element *elem)
{
	struct drm_encoder_element *prev = NULL;
	int ret;

	if (list_empty(&chain->elems))
		prev = list_last_entry(&chain->elems,
				       struct drm_encoder_element, node);

	if (prev && prev->funcs->attach) {
		ret = prev->funcs->attach(prev, elem);
		if (ret)
			return ret;
	}

	list_add_tail(&elem->node, &chain->elems);
	elem->chain = chain;

	return 0;
}
EXPORT_SYMBOL(drm_encoder_chain_attach_element);

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
