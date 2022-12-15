/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the USB device Suspend and Resume
 *              Application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB_CDC.h"
#include <stdbool.h>
#include <string.h>

/*Macros*/
/* Defines how often a message is printed in milliseconds */
#define MESSAGE_PRINT_PERIOD         (3000u)
/* Timer period in millisecond */
#define TIMER_PERIOD_MS              (1u)
/* Timer frequency in Hz */
#define TIMER_FREQ_HZ                (1000000u)
/* Timer cycles to achieve the desired period */
#define TIMER_PERIOD_CYCLE           ((TIMER_FREQ_HZ) / (TIMER_PERIOD_MS) / (1000u))


/* Function Prototypes*/
static void timer_callback(void *arg, cyhal_timer_event_t event);
static void wakeup_pin_isr(void);
static bool deepsleep_callback(cyhal_syspm_callback_state_t state,
        cyhal_syspm_callback_mode_t mode, void *callback_arg);


/*Global Variables*/
static const USB_DEVICE_INFO usb_deviceInfo =
{
        0x058B,                   /* VendorId */
        0x027A,                   /* ProductId */
        "Infineon Technologies",  /* VendorName */
        "CDC Code Example",       /* ProductName */
        "12345678"                /* SerialNumber */
};

/* USB wakeup pin interrupt configuration */
static cy_stc_sysint_t wakeup_pin_interrupt_cfg =
{
        .intrSrc = ioss_interrupts_gpio_14_IRQn,
        .intrPriority = 0,
};

static USB_CDC_HANDLE hInst;
#ifdef COMPONENT_CAT1A
static U8 usb_out_buffer[USB_FS_BULK_MAX_PACKET_SIZE];
#else
static U8 usb_out_buffer[USB_HS_BULK_MAX_PACKET_SIZE];
#endif

cyhal_timer_t timer_obj;

static volatile bool usb_suspend_flag = false;
static volatile uint8_t usb_idle_counter = 0u;
static volatile uint16_t usb_msg_counter = 0u;
static volatile bool usb_resume_flag  = false;


#if (USBD_NORTOS_TICKCNT_ENABLE == 0u)

static cyhal_timer_t usbd_timer_obj;
static uint32_t      timer_tick_count;

/********************************************************************************
 * Function Name: isr_timer
 ********************************************************************************
 * Summary:
 *  Local redefinition of the interrupt service routine for emUSB-Device
 *  middleware's timer. Increments value every ms.
 *
 * Parameters:
 *  callback_arg: not used
 *  event: note used
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    CY_UNUSED_PARAMETER(callback_arg);
    CY_UNUSED_PARAMETER(event);

    timer_tick_count++;
}

/********************************************************************************
 * Function Name: usbd_timer_config
 ********************************************************************************
 * Summary:
 *  Local redefinition of the timer configuration function for emUSB-Device
 *  middleware. This time is configured to generate interrupt at every 1 ms.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  cy_rslt_t: CY_RSLT_SUCCESS for successful timer configuration else error
 *             status
 *
 *******************************************************************************/
static cy_rslt_t usbd_timer_config(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Timer configuration structure */
    const cyhal_timer_cfg_t timer_cfg =
    {
            .compare_value = 0u, /* Set to 0, if not used */
            .period        = TIMER_PERIOD_CYCLE,
            .direction     = CYHAL_TIMER_DIR_UP,
            .is_compare    = false,
            .is_continuous = true,
            .value         = 0u,
    };

    /* Initialize the timer object without output pin and pre-configured clock source */
    result = cyhal_timer_init(&usbd_timer_obj, NC, NULL);

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_configure(&usbd_timer_obj, &timer_cfg);

        if (CY_RSLT_SUCCESS == result)
        {
            /* Set the frequency of timer to 1000000 Hz */
            result = cyhal_timer_set_frequency(&usbd_timer_obj, TIMER_FREQ_HZ);

            if (CY_RSLT_SUCCESS == result)
            {
                /* Assign the ISR to execute on timer interrupt */
                cyhal_timer_register_callback(&usbd_timer_obj, isr_timer, NULL);

                /* Set the event on which timer interrupt occurs and enable it */
                cyhal_timer_enable_event(&usbd_timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3U, true);

                /* Start the timer with the configured settings */
                result = cyhal_timer_start(&usbd_timer_obj);
            }
        }
    }
    return result;
}

/********************************************************************************
 * Function Name: USB_OS_GetTickCnt
 ********************************************************************************
 * Summary:
 *  Local redefinition of USB_OS_GetTickCnt.
 *  Returns the current system time in milliseconds or system ticks.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  U32: Current system time.
 *
 *******************************************************************************/
U32 USB_OS_GetTickCnt(void)
{
    return (U32) timer_tick_count;
}
#endif /* USBD_NORTOS_TICKCNT_ENABLE */

/********************************************************************************
 * Function Name: usb_add_cdc
 ********************************************************************************
 * Summary:
 *  Add CDC device to the emUSB-Device middleware.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void usb_add_cdc(void)
{
    USB_CDC_INIT_DATA initData;
    USB_ADD_EP_INFO   EPBulkIn;
    USB_ADD_EP_INFO   EPBulkOut;
    USB_ADD_EP_INFO   EPIntIn;

    memset(&initData, 0, sizeof(initData));
    memset(&EPBulkIn, 0, sizeof(EPBulkIn));
    memset(&EPBulkOut, 0, sizeof(EPBulkOut));
    memset(&EPIntIn, 0, sizeof(EPIntIn));


    /* Bulk In endpoint descriptor */
    EPBulkIn.Flags          = 0u;                             /* Flags not used. */
    EPBulkIn.InDir          = USB_DIR_IN;                     /* IN direction (Device to Host) */
    EPBulkIn.Interval       = 0u;                             /* Interval not used for Bulk endpoints. */
#ifdef COMPONENT_CAT1A
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;    /* Maximum packet size (64 for Bulk in full-speed). */
#else
    EPBulkIn.MaxPacketSize  = USB_HS_BULK_MAX_PACKET_SIZE;    /* Maximum packet size (512 for Bulk in high-speed). */
#endif
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;         /* Endpoint type - Bulk. */

    /* Add Bulk In endpoint */
    initData.EPIn           = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    /* Bulk Out endpoint descriptor */
    EPBulkOut.Flags         = 0u;                             /* Flags not used. */
    EPBulkOut.InDir         = USB_DIR_OUT;                    /* OUT direction (Host to Device) */
    EPBulkOut.Interval      = 0u;                             /* Interval not used for Bulk endpoints. */
#ifdef COMPONENT_CAT1A
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;    /* Maximum packet size (64 for Bulk out full-speed). */
#else
    EPBulkOut.MaxPacketSize = USB_HS_BULK_MAX_PACKET_SIZE;    /* Maximum packet size (512 for Bulk out high-speed). */
#endif
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;         /* Endpoint type - Bulk. */

    /* Add Bulk Out endpoint */
    initData.EPOut          = USBD_AddEPEx(&EPBulkOut, usb_out_buffer, sizeof(usb_out_buffer));

    /* Interrupt In endpoint descriptor */
    EPIntIn.Flags           = 0u;                             /* Flags not used. */
    EPIntIn.InDir           = USB_DIR_IN;                     /* IN direction (Device to Host) */

    EPIntIn.Interval        = 8u;                             /* Interval of 1 ms (125 us * 8) */
#ifdef COMPONENT_CAT1A
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE;     /* Maximum packet size (64 for Interrupt in full-speed). */
#else
    EPIntIn.MaxPacketSize   = USB_HS_INT_MAX_PACKET_SIZE;     /* Maximum packet size (512 for Interrupt in high-speed). */
#endif
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;          /* Endpoint type - Interrupt. */

    /* Add Interrupt In endpoint */
    initData.EPInt          = USBD_AddEPEx(&EPIntIn, NULL, 0);

    /* Adds a CDC device class to the USB middleware */
    hInst = USBD_CDC_Add(&initData);
}

/********************************************************************************
 * Function Name: print_message
 ********************************************************************************
 * Summary:
 *  Print a message to the COM terminal over USB CDC.
 *
 * Parameters:
 *  msg: msg to be written on USB CDC port.
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void print_message(const char msg[])
{
    int msg_len = strlen(msg);
    if (msg_len < sizeof(usb_out_buffer))
    {
        memset(usb_out_buffer, 0, sizeof(usb_out_buffer));
        memcpy(usb_out_buffer, (U8*)msg, msg_len);
        USBD_CDC_Write(hInst, &usb_out_buffer, sizeof(usb_out_buffer), 100);
    }
}

/********************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  This is the main function for CM4 CPU. It initializes the USB device block and
 *  enumerates as a CDC device. When the  USB suspend condition is detected, it
 *  sends the device to a low power state, and restores normal operation when
 *  USB activity resumes.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    cyhal_syspm_callback_data_t syspm_callback_data =
    {
            deepsleep_callback,
            CYHAL_SYSPM_CB_CPU_DEEPSLEEP,
            (cyhal_syspm_callback_mode_t) 0,
            NULL,
            NULL
    };

    const cyhal_timer_cfg_t timer_cfg =
    {
            .compare_value = 0u,
            .period        = TIMER_PERIOD_CYCLE,
            .direction     = CYHAL_TIMER_DIR_UP,
            .is_compare    = false,
            .is_continuous = true,
            .value = 0u
    };

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "emUSB-Device: Suspend and resume"
           "****************** \r\n\n");

    /* Enable global interrupts */
    __enable_irq();

    /* Power Management callback registration */
    cyhal_syspm_register_callback(&syspm_callback_data);

    /* Initialize the Kit user LED to depict ACTIVE/DEEPSLEEP state */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
        CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);


#ifdef COMPONENT_CAT1A
    /* Initialize the USB wakeup interrupt */
    if (CY_RSLT_SUCCESS != Cy_SysInt_Init(&wakeup_pin_interrupt_cfg, &wakeup_pin_isr))
    {
        CY_ASSERT(0);
    }

    /* Setup USB Dp pin to generate an interrupt on falling edge */
    /* This interrupt wakes up the CPU from Deep-Sleep */
    Cy_GPIO_SetInterruptEdge(GPIO_PRT14, 0u, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(GPIO_PRT14, 0u, 1UL);
#endif


    /* Initialize the application timer */
    if (CY_RSLT_SUCCESS != cyhal_timer_init(&timer_obj, NC, NULL))
    {
        CY_ASSERT(0);
    }

    /* Apply timer configuration */
    if (CY_RSLT_SUCCESS != cyhal_timer_configure(&timer_obj, &timer_cfg))
    {
        CY_ASSERT(0);
    }

    /* Set the timer frequency to 1000000 Hz*/
    if (CY_RSLT_SUCCESS != cyhal_timer_set_frequency(&timer_obj, TIMER_FREQ_HZ))
    {
        CY_ASSERT(0);
    }

    /* Timer callback registration */
    cyhal_timer_register_callback(&timer_obj, timer_callback, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3u, true);

    /* Configure the timer for emUSB-Device middleware */
#if (USBD_NORTOS_TICKCNT_ENABLE == 0u)
    if (CY_RSLT_SUCCESS != usbd_timer_config())
    {
        CY_ASSERT(0);
    }
#endif /* USBD_NORTOS_TICKCNT_ENABLE */

    /* Initializes the USB device with its settings */
    USBD_Init();

    /* Add the CDC device */
    usb_add_cdc();

    /* Set USB device info to be used during device enumeration */
    USBD_SetDeviceInfo(&usb_deviceInfo);

    /* Starts the emUSB-Device Core */
    USBD_Start();

    /* Make device appear on the bus. This function call is blocking,
     * toggle the kit user LED until device gets enumerated.
     */
    while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED);
        cyhal_system_delay_ms(250);
    }

    /* Start the application timer */
    if (CY_RSLT_SUCCESS != cyhal_timer_start(&timer_obj))
    {
        CY_ASSERT(0);
    }

    /* Turn ON the kit user LED to indicate USB device is enumerated and working */
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

    for (;;)
    {
        /* Check if suspend condition is detected on the bus */
        if (usb_suspend_flag)
        {
            /* Power management mode: DeepSleep */
            USBD_Logf_Application("Device is going to suspend\r\n");

            /* Delay for printout character buffer */
            USB_OS_Delay(1000);


            /* Try to enter DEEPSLEEP mode */
            if (CY_RSLT_SUCCESS != cyhal_syspm_deepsleep())
            {
                printf("Entering DEEPSLEEP failed!\n\r");
            }
            else
            {
                if (usb_resume_flag)
                {
                    usb_resume_flag = false;
                    USBD_Logf_Application("Resume event from Host");

                    /* Resume recovery time */
                    USB_OS_Delay(10u);
                }

            }
        }
        else if (USB_STAT_CONFIGURED == (USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)))
        {
            /* Check if a message should be printed to the console */
            if (usb_msg_counter > MESSAGE_PRINT_PERIOD)
            {
                /* Reset message counter */
                usb_msg_counter = 0;

                /* Print message to the console */
                print_message("USB is active\n\r");
            }
        }
        else
        {
            /* Do Nothing */
        }
    }
}

/********************************************************************************
 * Function Name: timer_callback
 ********************************************************************************
 * Summary:
 *  One millisecond timer interrupt handler. Check for activity in the USB bus.
 *
 * Parameters:
 *  arg: not used
 *  event: not used
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void timer_callback(void *arg, cyhal_timer_event_t event)
{
    CY_UNUSED_PARAMETER(arg);
    CY_UNUSED_PARAMETER(event);

    /* Supervisor of suspend conditions on the bus */
#ifdef COMPONENT_CAT1A
    USB_DRIVER_Cypress_PSoC6_SysTick();
#endif

    if (USBD_GetState() & USB_STAT_SUSPENDED)
    {
        /* Suspend condition on USB bus is detected.
         * Request device to enter low-power mode
         */
        usb_suspend_flag = true;
    }
    else
    {
        /* Clear suspend conditions */
        usb_suspend_flag = false;
    }
    /* Counter to print USB message */
    usb_msg_counter++;
}


/********************************************************************************
 * Function Name: wake_pin_isr
 ********************************************************************************
 * Summary:
 *  Wake-up pin interrupt handler for USB Dp. Clear the interrupt only.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void wakeup_pin_isr(void)
{  
    /* Clear any pending interrupt */
    if (0u != Cy_GPIO_GetInterruptStatusMasked(GPIO_PRT14, 0u))
    {
        Cy_GPIO_ClearInterrupt(GPIO_PRT14, 0u);

        /* Resume event for emUSB stack */
        usb_resume_flag = true;
    }
}

/********************************************************************************
 * Function Name: deepsleep_callback
 ********************************************************************************
 * Summary:
 *  DEEPSLEEP callback implementation.
 *
 * Parameters:
 *  state - state the system or CPU is being transitioned into
 *  mode  - callback mode
 *  callback_arg - user argument (not used)
 *
 * Return:
 *  false - if transition is prohibited
 *  true if allowed
 *
 *******************************************************************************/
static bool deepsleep_callback(cyhal_syspm_callback_state_t state,
cyhal_syspm_callback_mode_t mode, void *callback_arg)
{
    CY_UNUSED_PARAMETER(state);
    CY_UNUSED_PARAMETER(callback_arg);
    bool status = true;

    switch (mode)
    {
        case CYHAL_SYSPM_CHECK_READY:
        {
            if (usb_suspend_flag) /* suspend detected */
            {
                /* Stop the application timer */
                if (CY_RSLT_SUCCESS == cyhal_timer_stop(&timer_obj))
                {
#if (USBD_NORTOS_TICKCNT_ENABLE == 0)
                    /* Stop USB Timer */
                    if (CY_RSLT_SUCCESS != cyhal_timer_stop(&usbd_timer_obj))
                    {
                        status = false;
                    }
#endif /* USBD_NORTOS_TICKCNT_ENABLE */
                }
                else
                {
                    status = false;
                }
            }
            break;
        }
        case CYHAL_SYSPM_CHECK_FAIL:
        {
            if (usb_suspend_flag)
            {
#if (USBD_NORTOS_TICKCNT_ENABLE == 0u)
                /* Restart USB Timer */
                if (CY_RSLT_SUCCESS != cyhal_timer_start(&usbd_timer_obj))
                {
                    CY_ASSERT(0);
                }
#endif /* USBD_NORTOS_TICKCNT_ENABLE */

                /* Start the application timer */
                if (CY_RSLT_SUCCESS != cyhal_timer_start(&timer_obj))
                {
                    CY_ASSERT(0);
                }
            }
            break;
        }
        case CYHAL_SYSPM_BEFORE_TRANSITION:
        {
            if (usb_suspend_flag)
            {
                Cy_GPIO_ClearInterrupt(GPIO_PRT14, 0u);

                /* Clear any pending interrupt */
                NVIC_ClearPendingIRQ(wakeup_pin_interrupt_cfg.intrSrc);

                /* Enable the Wake-up interrupt pin */
                NVIC_EnableIRQ(wakeup_pin_interrupt_cfg.intrSrc);

                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            }
            break;
        }
        case CY_SYSPM_AFTER_TRANSITION:
        {       
            /* Disable the Wake-up interrupt pin */
            NVIC_DisableIRQ(wakeup_pin_interrupt_cfg.intrSrc);

#if (USBD_NORTOS_TICKCNT_ENABLE == 0)
            /* Start USB Timer */
            if (CY_RSLT_SUCCESS != cyhal_timer_start(&usbd_timer_obj))
            {
                CY_ASSERT(0);
            }
#endif /* USBD_NORTOS_TICKCNT_ENABLE */

            /* Resume the emUSB Device */
#ifdef COMPONENT_CAT1A
            USB_DRIVER_Cypress_PSoC6_Resume();
#endif          
            /* Turn ON the kit user LED to indicate that the USB is in active mode */
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

            /* Start the application timer */
            if (CY_RSLT_SUCCESS != cyhal_timer_start(&timer_obj))
            {
                CY_ASSERT(0);
            }
            break;
        }
        default:
        {
            /* Should NOT happen */
            CY_ASSERT(0);
            break;          
        }
    }

    return status;
}

/* [] END OF FILE */
