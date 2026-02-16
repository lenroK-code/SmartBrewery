#pragma once

void screen_init(void);
// Update displayed values: specific gravity (sg), Brix/Plato (blg), temperature in °C
void eink_update_values(float sg, float blg, float temp_c);