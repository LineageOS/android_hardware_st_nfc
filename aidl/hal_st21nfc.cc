/******************************************************************************
 *
 *  Copyright (C) 1999-2012 Broadcom Corporation
 *  Copyright (C) 2013 ST Microelectronics S.A.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Modified by ST Microelectronics S.A. (adaptation of nfc_nci.c for ST21NFC
 *NCI version)
 *
 ******************************************************************************/

#include <android-base/properties.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>

#include "StNfc_hal_api.h"
#include "android_logmsg.h"
#include "hal_config.h"
#include "hal_fd.h"
#include "halcore.h"
#include "st21nfc_dev.h"

#if defined(ST_LIB_32)
#define VENDOR_LIB_PATH "/vendor/lib/"
#else
#define VENDOR_LIB_PATH "/vendor/lib64/"
#endif
#define VENDOR_LIB_EXT ".so"

#define CRC_PRESET_A 0x6363
#define CRC_PRESET_B 0xFFFF
#define Type_A 0
#define Type_B 1

bool dbg_logging = false;

extern void HalCoreCallback(void* context, uint32_t event, const void* d,
                            size_t length);
extern bool I2cOpenLayer(void* dev, HAL_CALLBACK callb, HALHANDLE* pHandle);

typedef int (*STEseReset)(void);

const char* halVersion = "ST21NFC AIDL Version 1.0.0";

uint8_t cmd_set_nfc_mode_enable[] = {0x2f, 0x02, 0x02, 0x02, 0x01};
uint8_t hal_is_closed = 1;
pthread_mutex_t hal_mtx = PTHREAD_MUTEX_INITIALIZER;
st21nfc_dev_t dev;
int nfc_mode = 0;
uint8_t nci_cmd[256];

/*
 * NCI HAL method implementations. These must be overridden
 */

extern bool hal_wrapper_open(st21nfc_dev_t* dev, nfc_stack_callback_t* p_cback,
                             nfc_stack_data_callback_t* p_data_cback,
                             HALHANDLE* pHandle);

extern int hal_wrapper_close(int call_cb, int nfc_mode);

extern void hal_wrapper_send_config();
extern void hal_wrapper_factoryReset();
extern void hal_wrapper_set_observer_mode(uint8_t enable, bool per_tech_cmd);
extern void hal_wrapper_get_observer_mode();

/* Make sure to always post nfc_stack_callback_t in a separate thread.
This prevents a possible deadlock in upper layer on some sequences.
We need to synchronize finely for the callback called for hal close,
otherwise the upper layer either does not receive the event, or deadlocks,
because the HAL is closing while the callback may be blocked.
 */
static struct async_callback_struct {
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  pthread_t thr;
  int event_pending;
  int stop_thread;
  int thread_running;
  nfc_event_t event;
  nfc_status_t event_status;
} async_callback_data;

static void* async_callback_thread_fct(void* arg) {
  int ret;
  struct async_callback_struct* pcb_data = (struct async_callback_struct*)arg;

  ret = pthread_mutex_lock(&pcb_data->mutex);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_mutex_lock failed", __func__);
    goto error;
  }

  do {
    if (pcb_data->event_pending == 0) {
      ret = pthread_cond_wait(&pcb_data->cond, &pcb_data->mutex);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_cond_wait failed", __func__);
        break;
      }
    }

    if (pcb_data->event_pending) {
      nfc_event_t event = pcb_data->event;
      nfc_status_t event_status = pcb_data->event_status;
      int ending = pcb_data->stop_thread;
      pcb_data->event_pending = 0;
      ret = pthread_cond_signal(&pcb_data->cond);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_cond_signal failed", __func__);
        break;
      }
      if (ending) {
        pcb_data->thread_running = 0;
      }
      ret = pthread_mutex_unlock(&pcb_data->mutex);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_mutex_unlock failed", __func__);
      }
      STLOG_HAL_D("HAL st21nfc: %s event %hhx status %hhx", __func__, event,
                  event_status);
      dev.p_cback_unwrap(event, event_status);
      if (ending) {
        return NULL;
      }
      ret = pthread_mutex_lock(&pcb_data->mutex);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_mutex_lock failed", __func__);
        goto error;
      }
    }
  } while (pcb_data->stop_thread == 0 || pcb_data->event_pending);

  ret = pthread_mutex_unlock(&pcb_data->mutex);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_mutex_unlock failed", __func__);
  }

error:
  pcb_data->thread_running = 0;
  return NULL;
}

static int async_callback_thread_start() {
  int ret;

  memset(&async_callback_data, 0, sizeof(async_callback_data));

  ret = pthread_mutex_init(&async_callback_data.mutex, NULL);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_mutex_init failed", __func__);
    return ret;
  }

  ret = pthread_cond_init(&async_callback_data.cond, NULL);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_cond_init failed", __func__);
    return ret;
  }

  async_callback_data.thread_running = 1;

  ret = pthread_create(&async_callback_data.thr, NULL,
                       async_callback_thread_fct, &async_callback_data);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_create failed", __func__);
    async_callback_data.thread_running = 0;
    return ret;
  }

  return 0;
}

static int async_callback_thread_end() {
  if (async_callback_data.thread_running != 0) {
    int ret;

    ret = pthread_mutex_lock(&async_callback_data.mutex);
    if (ret != 0) {
      STLOG_HAL_E("HAL: %s pthread_mutex_lock failed", __func__);
      return ret;
    }

    async_callback_data.stop_thread = 1;

    // Wait for the thread to have no event pending
    while (async_callback_data.thread_running &&
           async_callback_data.event_pending) {
      ret = pthread_cond_signal(&async_callback_data.cond);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_cond_signal failed", __func__);
        return ret;
      }
      ret = pthread_cond_wait(&async_callback_data.cond,
                              &async_callback_data.mutex);
      if (ret != 0) {
        STLOG_HAL_E("HAL: %s pthread_cond_wait failed", __func__);
        break;
      }
    }

    ret = pthread_mutex_unlock(&async_callback_data.mutex);
    if (ret != 0) {
      STLOG_HAL_E("HAL: %s pthread_mutex_unlock failed", __func__);
      return ret;
    }

    ret = pthread_cond_signal(&async_callback_data.cond);
    if (ret != 0) {
      STLOG_HAL_E("HAL: %s pthread_cond_signal failed", __func__);
      return ret;
    }

    ret = pthread_join(async_callback_data.thr, (void**)NULL);
    if (ret != 0) {
      STLOG_HAL_E("HAL: %s pthread_join failed", __func__);
      return ret;
    }
  }
  return 0;
}

static void async_callback_post(nfc_event_t event, nfc_status_t event_status) {
  int ret;

  if (pthread_equal(pthread_self(), async_callback_data.thr)) {
    dev.p_cback_unwrap(event, event_status);
  }

  ret = pthread_mutex_lock(&async_callback_data.mutex);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_mutex_lock failed", __func__);
    return;
  }

  if (async_callback_data.thread_running == 0) {
    (void)pthread_mutex_unlock(&async_callback_data.mutex);
    STLOG_HAL_E("HAL: %s thread is not running", __func__);
    dev.p_cback_unwrap(event, event_status);
    return;
  }

  while (async_callback_data.event_pending) {
    ret = pthread_cond_wait(&async_callback_data.cond,
                            &async_callback_data.mutex);
    if (ret != 0) {
      STLOG_HAL_E("HAL: %s pthread_cond_wait failed", __func__);
      return;
    }
  }

  async_callback_data.event_pending = 1;
  async_callback_data.event = event;
  async_callback_data.event_status = event_status;

  ret = pthread_mutex_unlock(&async_callback_data.mutex);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_mutex_unlock failed", __func__);
    return;
  }

  ret = pthread_cond_signal(&async_callback_data.cond);
  if (ret != 0) {
    STLOG_HAL_E("HAL: %s pthread_cond_signal failed", __func__);
    return;
  }
}
/* ------ */

int StNfc_hal_open(nfc_stack_callback_t* p_cback,
                   nfc_stack_data_callback_t* p_data_cback) {
  bool result = false;

  STLOG_HAL_D("HAL st21nfc: %s %s", __func__, halVersion);

  (void)pthread_mutex_lock(&hal_mtx);

  if (!hal_is_closed) {
    hal_wrapper_close(0, nfc_mode);
  }

  dev.p_cback = p_cback;  // will be replaced by wrapper version
  dev.p_cback_unwrap = p_cback;
  dev.p_data_cback = p_data_cback;
  // Initialize and get global logging level
  InitializeSTLogLevel();

  if ((hal_is_closed || !async_callback_data.thread_running) &&
      (async_callback_thread_start() != 0)) {
    dev.p_cback(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_FAILED);
    (void)pthread_mutex_unlock(&hal_mtx);
    return -1;  // We are doomed, stop it here, NOW !
  }
  result =
      hal_wrapper_open(&dev, async_callback_post, p_data_cback, &(dev.hHAL));

  if (!result || !(dev.hHAL)) {
    async_callback_post(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_FAILED);
    (void)pthread_mutex_unlock(&hal_mtx);
    return -1;  // We are doomed, stop it here, NOW !
  }
  hal_is_closed = 0;
  (void)pthread_mutex_unlock(&hal_mtx);
  return 0;
}

int StNfc_hal_write(uint16_t data_len, const uint8_t* p_data) {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);

  uint8_t NCI_ANDROID_PASSIVE_OBSERVER_PREFIX[] = {0x2f, 0x0c, 0x02, 0x02};
  uint8_t NCI_ANDROID_PASSIVE_OBSERVER_PER_TECH_PREFIX[] = {0x2f, 0x0c, 0x02,
                                                            0x05};
  uint8_t NCI_QUERY_ANDROID_PASSIVE_OBSERVER_PREFIX[] = {0x2f, 0x0c, 0x01, 0x4};
  uint8_t NCI_ANDROID_PREFIX[] = {0x2f, 0x0c};
  uint8_t RF_GET_LISTEN_OBSERVE_MODE_STATE[5] = {0x21, 0x17, 0x00};
  uint8_t RF_SET_LISTEN_OBSERVE_MODE_STATE[4] = {0x21, 0x16, 0x01, 0x0};
  uint8_t CORE_GET_CONFIG_OBSERVER[5] = {0x20, 0x03, 0x02, 0x01, 0xa3};
  uint8_t CORE_SET_CONFIG_OBSERVER[7] = {0x20, 0x02, 0x04, 0x01,
                                         0xa3, 0x01, 0x00};
  uint8_t* mGetObserve = CORE_GET_CONFIG_OBSERVER;
  uint8_t mGetObserve_size = 5;
  uint8_t* mSetObserve = CORE_SET_CONFIG_OBSERVER;
  uint8_t mSetObserve_size = 7;
  uint8_t mTechObserved = 0x0;
  /* check if HAL is closed */
  int ret = (int)data_len;
  (void)pthread_mutex_lock(&hal_mtx);
  if (hal_is_closed) {
    ret = 0;
  }

  if (!ret) {
    (void)pthread_mutex_unlock(&hal_mtx);
    return ret;
  }

  if (data_len == 4 &&
      !memcmp(p_data, NCI_QUERY_ANDROID_PASSIVE_OBSERVER_PREFIX,
              sizeof(NCI_QUERY_ANDROID_PASSIVE_OBSERVER_PREFIX))) {
    hal_wrapper_get_observer_mode();
    if (hal_fd_getFwCap()->ObserveMode == 2) {
      mGetObserve = RF_GET_LISTEN_OBSERVE_MODE_STATE;
      mGetObserve_size = 3;
    }
    if (!HalSendDownstream(dev.hHAL, mGetObserve, mGetObserve_size)) {
      STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
  }

  else if (data_len == 5 &&
           !memcmp(p_data, NCI_ANDROID_PASSIVE_OBSERVER_PREFIX,
                   sizeof(NCI_ANDROID_PASSIVE_OBSERVER_PREFIX))) {
    if (hal_fd_getFwCap()->ObserveMode == 2) {
      mSetObserve = RF_SET_LISTEN_OBSERVE_MODE_STATE;
      mSetObserve_size = 4;
      if (p_data[4]) {
        mTechObserved = 0x7;
      }
      mSetObserve[3] = mTechObserved;
      hal_wrapper_set_observer_mode(mTechObserved, false);
    } else {
      mSetObserve[6] = p_data[4];
      hal_wrapper_set_observer_mode(p_data[4], false);
    }

    if (!HalSendDownstream(dev.hHAL, mSetObserve, mSetObserve_size)) {
      STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
  } else if (data_len == 5 &&
             !memcmp(p_data, NCI_ANDROID_PASSIVE_OBSERVER_PER_TECH_PREFIX,
                     sizeof(NCI_ANDROID_PASSIVE_OBSERVER_PER_TECH_PREFIX))) {
    mSetObserve = RF_SET_LISTEN_OBSERVE_MODE_STATE;
    mSetObserve_size = 4;
    if (p_data[4]) {
      mTechObserved = p_data[4];
    }
    mSetObserve[3] = mTechObserved;
    hal_wrapper_set_observer_mode(mTechObserved, true);
    if (!HalSendDownstream(dev.hHAL, mSetObserve, mSetObserve_size)) {
      STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
  } else if (!memcmp(p_data, NCI_ANDROID_PREFIX, sizeof(NCI_ANDROID_PREFIX)) &&
             p_data[3] == 0x6) {
    DispHal("TX DATA", (p_data), data_len);

    memcpy(nci_cmd + 3, p_data + 4, 4);
    nci_cmd[0] = 0x2f;
    nci_cmd[1] = 0x19;

    int index = 8;
    int ll_index = 7;
    uint8_t nci_length = 0;
    uint16_t crc = 0;
    bool prefix_match = false;
    bool exact_match = true;

    while (index < data_len) {
      // Read the Type field (1 byte)
      uint8_t type_field = p_data[index];
      int tlv_len = p_data[index + 1];
      prefix_match = false;
      exact_match = true;
      if (p_data[index] == 0x01) {
        crc = iso14443_crc(p_data + index + 3, (uint8_t)((tlv_len - 1) / 2),
                           Type_B);
      } else if ((p_data[index] & 0xF0) == 0x00) {
        crc = iso14443_crc(p_data + index + 3, (uint8_t)((tlv_len - 1) / 2),
                           Type_A);
      } else {
        prefix_match = true;
      }

      nci_cmd[ll_index++] = p_data[index++];
      nci_cmd[ll_index++] =
          (!prefix_match) ? p_data[index++] + 4 : p_data[index++];
      nci_cmd[ll_index++] = p_data[index++];

      memcpy(nci_cmd + ll_index, p_data + index, (uint8_t)((tlv_len - 1) / 2));
      ll_index += (tlv_len - 1) / 2;
      index += (tlv_len - 1) / 2;
      int crc_index = 0;
      if (!prefix_match) {
        crc_index = ll_index;
        nci_cmd[ll_index++] = (uint8_t)crc;
        nci_cmd[ll_index++] = (uint8_t)(crc >> 8);
      }

      memcpy(nci_cmd + ll_index, p_data + index, (tlv_len - 1) / 2);
      for (int i = 0; i < (tlv_len - 1) / 2; ++i) {
        if (p_data[index + i] != 0xFF) {
          exact_match = false;
          break;
        }
      }
      ll_index += (tlv_len - 1) / 2;
      index += (tlv_len - 1) / 2;
      uint8_t crc_mask = exact_match ? 0xFF : 0x00;
      if (!prefix_match) {
        nci_cmd[ll_index++] = crc_mask;
        nci_cmd[ll_index++] = crc_mask;

        if (!exact_match) {
          nci_cmd[crc_index] = crc_mask;
          nci_cmd[crc_index + 1] = crc_mask;
        }
      }
    }
    nci_length = ll_index;
    nci_cmd[2] = ll_index - 3;

    if (!HalSendDownstream(dev.hHAL, nci_cmd, nci_length)) {
      STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
  } else if (!memcmp(p_data, NCI_ANDROID_PREFIX, sizeof(NCI_ANDROID_PREFIX)) &&
             p_data[3] == 0x9) {
    DispHal("TX DATA", (p_data), data_len);
    if (data_len < 5) {
      STLOG_HAL_E("HAL st21nfc %s  data_len is too short", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
    memcpy(nci_cmd + 3, p_data + 4, data_len - 4);
    nci_cmd[0] = 0x2f;
    nci_cmd[1] = 0x1d;
    if (p_data[2] == 0x2 && p_data[4] == 0x0) {
      nci_cmd[2] = 0x1;
    } else {
      uint16_t crc = 0;
      if (nci_cmd[5] > 0) {
        crc = iso14443_crc(nci_cmd + 7, nci_cmd[5] - 1, Type_A);
      }

      uint8_t len = p_data[2];
      nci_cmd[5] = nci_cmd[5] + 2;
      nci_cmd[data_len - 1] = (uint8_t)crc;
      nci_cmd[data_len] = (uint8_t)(crc >> 8);

      nci_cmd[2] = p_data[2] + 1;
    }
    if (!HalSendDownstream(dev.hHAL, nci_cmd, nci_cmd[2] + 3)) {
      STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
      (void)pthread_mutex_unlock(&hal_mtx);
      return 0;
    }
  } else if (!HalSendDownstream(dev.hHAL, p_data, data_len)) {
    STLOG_HAL_E("HAL st21nfc %s  SendDownstream failed", __func__);
    (void)pthread_mutex_unlock(&hal_mtx);
    return 0;
  }
  (void)pthread_mutex_unlock(&hal_mtx);

  return ret;
}

int StNfc_hal_core_initialized() {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);

  (void)pthread_mutex_lock(&hal_mtx);
  hal_wrapper_send_config();
  (void)pthread_mutex_unlock(&hal_mtx);

  return 0;  // return != 0 to signal ready immediate
}

int StNfc_hal_pre_discover() {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);
  async_callback_post(HAL_NFC_PRE_DISCOVER_CPLT_EVT, HAL_NFC_STATUS_OK);
  // callback directly if no vendor-specific pre-discovery actions are needed
  return 0;
}

int StNfc_hal_close(int nfc_mode_value) {
  void* stdll = nullptr;
  STLOG_HAL_D("HAL st21nfc: %s nfc_mode = %d", __func__, nfc_mode_value);

  /* check if HAL is closed */
  (void)pthread_mutex_lock(&hal_mtx);
  if (hal_is_closed) {
    (void)pthread_mutex_unlock(&hal_mtx);
    return 1;
  }
  if (hal_wrapper_close(1, nfc_mode_value) == -1) {
    hal_is_closed = 1;
    (void)pthread_mutex_unlock(&hal_mtx);
    return 1;
  }
  hal_is_closed = 1;
  (void)pthread_mutex_unlock(&hal_mtx);

  deInitializeHalLog();

  if (async_callback_thread_end() != 0) {
    STLOG_HAL_E("HAL st21nfc: %s async_callback_thread_end failed", __func__);
    return -1;  // We are doomed, stop it here, NOW !
  }

  std::string valueStr =
      android::base::GetProperty("persist.vendor.nfc.streset", "");
  // do a cold_reset when nfc is off
  if (valueStr.length() > 0 && nfc_mode_value == 0) {
    stdll = dlopen(valueStr.c_str(), RTLD_NOW);
    if (!stdll) {
      valueStr = VENDOR_LIB_PATH + valueStr + VENDOR_LIB_EXT;
      stdll = dlopen(valueStr.c_str(), RTLD_NOW);
    }
    if (stdll) {
      STLOG_HAL_D("STReset Cold reset");
      STEseReset fn = (STEseReset)dlsym(stdll, "cold_reset");
      if (fn) {
        int ret = fn();
        STLOG_HAL_D("STReset Result=%d", ret);
      }
    } else {
      STLOG_HAL_D("%s not found, do nothing.", valueStr.c_str());
    }
  }

  STLOG_HAL_D("HAL st21nfc: %s close", __func__);
  return 0;
}

int StNfc_hal_power_cycle() {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);

  /* check if HAL is closed */
  int ret = HAL_NFC_STATUS_OK;
  (void)pthread_mutex_lock(&hal_mtx);
  if (hal_is_closed) {
    ret = HAL_NFC_STATUS_FAILED;
  }

  if (ret != HAL_NFC_STATUS_OK) {
    (void)pthread_mutex_unlock(&hal_mtx);
    return ret;
  }
  async_callback_post(HAL_NFC_OPEN_CPLT_EVT, HAL_NFC_STATUS_OK);

  (void)pthread_mutex_unlock(&hal_mtx);
  return HAL_NFC_STATUS_OK;
}

void StNfc_hal_factoryReset() {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);
  // hal_wrapper_factoryReset();
  //  Nothing needed for factory reset in st21nfc case.
}

int StNfc_hal_closeForPowerOffCase() {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);
  if (nfc_mode == 1) {
    return 0;
  } else {
    return StNfc_hal_close(nfc_mode);
  }
}

void StNfc_hal_getConfig(NfcConfig& config) {
  STLOG_HAL_D("HAL st21nfc: %s", __func__);
  unsigned long num = 0;
  std::array<uint8_t, 10> buffer;

  buffer.fill(0);
  long retlen = 0;

  memset(&config, 0x00, sizeof(NfcConfig));

  if (GetNumValue(NAME_CE_ON_SWITCH_OFF_STATE, &num, sizeof(num))) {
    if (num == 0x1) {
      nfc_mode = 0x1;
    }
  }

  if (GetNumValue(NAME_POLL_BAIL_OUT_MODE, &num, sizeof(num))) {
    config.nfaPollBailOutMode = num;
  }

  if (GetNumValue(NAME_ISO_DEP_MAX_TRANSCEIVE, &num, sizeof(num))) {
    config.maxIsoDepTransceiveLength = num;
  }
  if (GetNumValue(NAME_DEFAULT_OFFHOST_ROUTE, &num, sizeof(num))) {
    config.defaultOffHostRoute = num;
  }
  if (GetNumValue(NAME_DEFAULT_NFCF_ROUTE, &num, sizeof(num))) {
    config.defaultOffHostRouteFelica = num;
  }
  if (GetNumValue(NAME_DEFAULT_SYS_CODE_ROUTE, &num, sizeof(num))) {
    config.defaultSystemCodeRoute = num;
  }
  if (GetNumValue(NAME_DEFAULT_SYS_CODE_PWR_STATE, &num, sizeof(num))) {
    config.defaultSystemCodePowerState = num;
  }
  if (GetNumValue(NAME_DEFAULT_ROUTE, &num, sizeof(num))) {
    config.defaultRoute = num;
  }
  if (GetByteArrayValue(NAME_DEVICE_HOST_ALLOW_LIST, (char*)buffer.data(),
                        buffer.size(), &retlen)) {
    config.hostAllowlist.resize(retlen);
    for (int i = 0; i < retlen; i++) {
      config.hostAllowlist[i] = buffer[i];
    }
  }

  if (GetNumValue(NAME_OFF_HOST_ESE_PIPE_ID, &num, sizeof(num))) {
    config.offHostESEPipeId = num;
  }
  if (GetNumValue(NAME_OFF_HOST_SIM_PIPE_ID, &num, sizeof(num))) {
    config.offHostSIMPipeId = num;
  }
  if ((GetByteArrayValue(NAME_NFA_PROPRIETARY_CFG, (char*)buffer.data(),
                         buffer.size(), &retlen)) &&
      (retlen == 9)) {
    config.nfaProprietaryCfg.protocol18092Active = (uint8_t)buffer[0];
    config.nfaProprietaryCfg.protocolBPrime = (uint8_t)buffer[1];
    config.nfaProprietaryCfg.protocolDual = (uint8_t)buffer[2];
    config.nfaProprietaryCfg.protocol15693 = (uint8_t)buffer[3];
    config.nfaProprietaryCfg.protocolKovio = (uint8_t)buffer[4];
    config.nfaProprietaryCfg.protocolMifare = (uint8_t)buffer[5];
    config.nfaProprietaryCfg.discoveryPollKovio = (uint8_t)buffer[6];
    config.nfaProprietaryCfg.discoveryPollBPrime = (uint8_t)buffer[7];
    config.nfaProprietaryCfg.discoveryListenBPrime = (uint8_t)buffer[8];
  } else {
    memset(&config.nfaProprietaryCfg, 0xFF, sizeof(ProtocolDiscoveryConfig));
  }
  if (GetNumValue(NAME_PRESENCE_CHECK_ALGORITHM, &num, sizeof(num))) {
    config.presenceCheckAlgorithm = (PresenceCheckAlgorithm)num;
  }

  if (GetNumValue(NAME_STNFC_USB_CHARGING_MODE, &num, sizeof(num))) {
    if ((num == 1) && (nfc_mode == 0x1)) {
      nfc_mode = 0x2;
    }
  }

  if (GetByteArrayValue(NAME_OFFHOST_ROUTE_UICC, (char*)buffer.data(),
                        buffer.size(), &retlen)) {
    config.offHostRouteUicc.resize(retlen);
    for (int i = 0; i < retlen; i++) {
      config.offHostRouteUicc[i] = buffer[i];
    }
  }

  if (GetByteArrayValue(NAME_OFFHOST_ROUTE_ESE, (char*)buffer.data(),
                        buffer.size(), &retlen)) {
    config.offHostRouteEse.resize(retlen);
    for (int i = 0; i < retlen; i++) {
      config.offHostRouteEse[i] = buffer[i];
    }
  }

  if (GetNumValue(NAME_DEFAULT_ISODEP_ROUTE, &num, sizeof(num))) {
    config.defaultIsoDepRoute = num;
  }
}

void StNfc_hal_setLogging(bool enable) {
  dbg_logging = enable;
  hal_wrapper_setFwLogging(enable);
  if (dbg_logging && hal_conf_trace_level < STNFC_TRACE_LEVEL_VERBOSE) {
    hal_trace_level = STNFC_TRACE_LEVEL_VERBOSE;
  } else {
    hal_trace_level = hal_conf_trace_level;
  }
}

bool StNfc_hal_isLoggingEnabled() { return dbg_logging; }

void StNfc_hal_dump(int fd) { hal_wrapper_dumplog(fd); }

uint16_t iso14443_crc(const uint8_t* data, size_t szLen, int type) {
  uint16_t tempCrc;
  if (type == Type_A) {
    tempCrc = (unsigned short)CRC_PRESET_A;
  } else {
    tempCrc = (unsigned short)CRC_PRESET_B;
  }
  do {
    uint8_t bt;
    bt = *data++;
    bt = (bt ^ (uint8_t)(tempCrc & 0x00FF));
    bt = (bt ^ (bt << 4));
    tempCrc = (tempCrc >> 8) ^ ((uint32_t)bt << 8) ^ ((uint32_t)bt << 3) ^
              ((uint32_t)bt >> 4);
  } while (--szLen);

  return tempCrc;
}
