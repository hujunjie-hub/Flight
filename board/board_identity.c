#include "board_identity.h"
#include "board.h"
#include <rtdevice.h>
#include <hw_rng.h>
#include "drv_crypto.h"

static uint64_t g_boot_id;
static uint32_t g_reset_flags;

uint64_t board_boot_id(void)
{
    return g_boot_id;
}

uint32_t board_reset_flags(void)
{
    return g_reset_flags;
}

#if defined(BSP_USING_USBD) && defined(RT_USB_DEVICE_CDC)
/* RT-Thread CDC uses the factory UID as the stable USB serial number. */
rt_err_t vcom_get_stored_serno(char *serial, int size)
{
    static const char hex[] = "0123456789ABCDEF";
    const uint32_t uid[] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    if (size < 24)
        return -RT_EINVAL;
    for (unsigned word = 0; word < 3; ++word)
        for (unsigned digit = 0; digit < 8; ++digit)
            serial[8u * word + digit] = hex[(uid[word] >> (28u - 4u * digit)) & 0xfu];
    return RT_EOK;
}
#endif

static int board_identity_init(void)
{
    g_reset_flags = RCC->RSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    rt_uint32_t words[2];
    struct rt_hwcrypto_ctx *rng = rt_hwcrypto_rng_create(rt_hwcrypto_dev_default());
    if (rng == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_err_t result = stm32_hwcrypto_rng_read(rng, &words[0]);
    if (result == RT_EOK)
    {
        result = stm32_hwcrypto_rng_read(rng, &words[1]);
    }
    rt_hwcrypto_rng_destroy(rng);
    if (result != RT_EOK)
    {
        return result;
    }
    g_boot_id = ((uint64_t)words[0] << 32) | words[1];
    return g_boot_id ? RT_EOK : -RT_ERROR;
}
INIT_COMPONENT_EXPORT(board_identity_init);
