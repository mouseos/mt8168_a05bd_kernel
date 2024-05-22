/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _PUBADC_MANAGE_H_
#define _PUBADC_MANAGE_H_

#include <linux/pinctrl/consumer.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

enum PUBADC_TYPE {
	PUBADC_NONE = 0,
	PUBADC_BAT_ID = 1,
	PUBADC_LIQUID = 2,
};

extern int pubadc_iio_read_channel(struct iio_channel *adc_ch, int *val, enum PUBADC_TYPE type);
bool get_pubadc_init_state(void);

#endif
