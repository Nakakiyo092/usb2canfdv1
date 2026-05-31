///////////////////////////////////////////////////////////////////////////////
// GNU General Public License v3.0
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Full license text: https://www.gnu.org/licenses/gpl-3.0.txt
// See also: LICENSE.md in the root of this repository
///////////////////////////////////////////////////////////////////////////////

#ifndef USB2CANFDV1_NVM_H
#define USB2CANFDV1_NVM_H

// Prototypes
void nvm_init(void);
HAL_StatusTypeDef nvm_get_serial_number(uint16_t *num);
HAL_StatusTypeDef nvm_update_serial_number(uint16_t num);

HAL_StatusTypeDef nvm_apply_startup_cfg(void);
HAL_StatusTypeDef nvm_update_startup_cfg(uint8_t mode);

#endif // USB2CANFDV1_NVM_H
