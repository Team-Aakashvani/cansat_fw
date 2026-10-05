#pragma once
#include <cstdio>
#define ESP_LOGI(t, ...) (printf("  I %s: ", t), printf(__VA_ARGS__), printf("\n"))
#define ESP_LOGW(t, ...) (printf("  W %s: ", t), printf(__VA_ARGS__), printf("\n"))
