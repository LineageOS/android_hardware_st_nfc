/*
 * Copyright (C) 2024 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <fuzzbinder/libbinder_ndk_driver.h>
#include <fuzzer/FuzzedDataProvider.h>

#include "Nfc.h"

using android::fuzzService;
using ndk::SharedRefBase;
using ::aidl::android::hardware::nfc::Nfc;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  std::shared_ptr<Nfc> nfc_service = ndk::SharedRefBase::make<Nfc>();
  fuzzService(nfc_service->asBinder().get(), FuzzedDataProvider(data, size));
  return 0;
}