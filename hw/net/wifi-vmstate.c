/*
 * Migration helpers for the Wi-Fi chip models
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "migration/qemu-file-types.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "wifi-vmstate.h"

/* what a chip's queues and tables hold is bounded well below these */
#define MSGQ_MAX        4096
#define MSG_MAX         (1 << 20)
#define U32MAP_MAX      (1 << 20)

static void wifi_put_bytes(QEMUFile *f, GByteArray *m)
{
    qemu_put_be32(f, m->len);
    qemu_put_buffer(f, m->data, m->len);
}

static GByteArray *wifi_get_bytes(QEMUFile *f, Error **errp)
{
    uint32_t len = qemu_get_be32(f);
    GByteArray *m;

    if (len > MSG_MAX) {
        error_setg(errp, "wifi: message of %u bytes", len);
        return NULL;
    }
    m = g_byte_array_sized_new(len);
    g_byte_array_set_size(m, len);
    if (qemu_get_buffer(f, m->data, len) != len) {
        g_byte_array_unref(m);
        error_setg(errp, "wifi: short message");
        return NULL;
    }
    return m;
}

static bool msgq_save(QEMUFile *f, void *pv, size_t size,
                      const VMStateField *field, JSONWriter *vmdesc,
                      Error **errp)
{
    GQueue *q = pv;

    qemu_put_be32(f, g_queue_get_length(q));
    for (GList *l = q->head; l; l = l->next) {
        wifi_put_bytes(f, l->data);
    }
    return true;
}

static bool msgq_load(QEMUFile *f, void *pv, size_t size,
                      const VMStateField *field, Error **errp)
{
    GQueue *q = pv;
    uint32_t n = qemu_get_be32(f);
    GByteArray *m;

    if (n > MSGQ_MAX) {
        error_setg(errp, "wifi: %u queued messages", n);
        return false;
    }
    while ((m = g_queue_pop_head(q))) {
        g_byte_array_unref(m);
    }
    while (n--) {
        m = wifi_get_bytes(f, errp);
        if (!m) {
            return false;
        }
        g_queue_push_tail(q, m);
    }
    return true;
}

const VMStateInfo vmstate_info_wifi_msgq = {
    .name = "wifi_msgq",
    .load = msgq_load,
    .save = msgq_save,
};

static bool bytes_save(QEMUFile *f, void *pv, size_t size,
                       const VMStateField *field, JSONWriter *vmdesc,
                       Error **errp)
{
    GByteArray *m = *(GByteArray **)pv;

    qemu_put_byte(f, m != NULL);
    if (m) {
        wifi_put_bytes(f, m);
    }
    return true;
}

static bool bytes_load(QEMUFile *f, void *pv, size_t size,
                       const VMStateField *field, Error **errp)
{
    GByteArray **mp = pv;

    g_clear_pointer(mp, g_byte_array_unref);
    if (qemu_get_byte(f)) {
        *mp = wifi_get_bytes(f, errp);
        return *mp != NULL;
    }
    return true;
}

const VMStateInfo vmstate_info_wifi_bytes = {
    .name = "wifi_bytes",
    .load = bytes_load,
    .save = bytes_save,
};

static bool u32map_save(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field, JSONWriter *vmdesc,
                        Error **errp)
{
    GHashTable *t = *(GHashTable **)pv;
    GHashTableIter it;
    gpointer k, v;

    qemu_put_be32(f, g_hash_table_size(t));
    g_hash_table_iter_init(&it, t);
    while (g_hash_table_iter_next(&it, &k, &v)) {
        qemu_put_be32(f, GPOINTER_TO_UINT(k));
        qemu_put_be32(f, GPOINTER_TO_UINT(v));
    }
    return true;
}

static bool u32map_load(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field, Error **errp)
{
    GHashTable *t = *(GHashTable **)pv;
    uint32_t n = qemu_get_be32(f);

    if (n > U32MAP_MAX) {
        error_setg(errp, "wifi: %u table entries", n);
        return false;
    }
    g_hash_table_remove_all(t);
    while (n--) {
        uint32_t k = qemu_get_be32(f);
        uint32_t v = qemu_get_be32(f);

        g_hash_table_insert(t, GUINT_TO_POINTER(k), GUINT_TO_POINTER(v));
    }
    return true;
}

const VMStateInfo vmstate_info_wifi_u32map = {
    .name = "wifi_u32map",
    .load = u32map_load,
    .save = u32map_save,
};

const VMStateDescription vmstate_wifi_mac = {
    .name = "wifi-mac",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(a, MACAddr, 6),
        VMSTATE_END_OF_LIST()
    },
};
