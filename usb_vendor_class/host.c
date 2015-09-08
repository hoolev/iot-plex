#include <unistd.h>
#include <stdio.h>
#include "libusb-1.0/libusb.h"

#define VID 1003
#define PID 9251

//! Size of buffer used for the loopback
#define  UDI_VENDOR_LOOPBACK_SIZE    1024

uint8_t udi_vendor_buf_out[UDI_VENDOR_LOOPBACK_SIZE];
uint8_t udi_vendor_buf_in[UDI_VENDOR_LOOPBACK_SIZE];

// the device's endpoints
static unsigned char udi_vendor_ep_interrupt_in ;
static unsigned char udi_vendor_ep_interrupt_out;
static unsigned char udi_vendor_ep_bulk_in      ;
static unsigned char udi_vendor_ep_bulk_out     ;
static unsigned char udi_vendor_ep_iso_in       ;
static unsigned char udi_vendor_ep_iso_out      ;

#define CALL_CHECK(fcall) do { r=fcall; if (r < 0) ERR_EXIT(r); } while (0);
#define ERR_EXIT(errcode) do { printf("   %s\n", libusb_strerror((enum libusb_error)errcode)); return -1; } while (0)

static void init_buffers(void)
{
	int i;
	// Fill buffer OUT
	for (i=0; i<sizeof(udi_vendor_buf_out); i+=4 ) {
		udi_vendor_buf_out[i+0] = (i>>24)&0xFF;
		udi_vendor_buf_out[i+1] = (i>>16)&0xFF;
		udi_vendor_buf_out[i+2] = (i>> 8)&0xFF;
		udi_vendor_buf_out[i+3] = (i>> 0)&0xFF;
	}
	// Reset buffer IN
	memset(udi_vendor_buf_in,0x55,sizeof(udi_vendor_buf_in));
}

static int cmp_buffers(void)
{
	if (0!=memcmp( udi_vendor_buf_out, udi_vendor_buf_in, sizeof(udi_vendor_buf_in)) ) {
		printf("!Wrong data! Error in loopback\n");
		return 1;
	}
	return 0;
}

static int loop_back_control(libusb_device_handle *device_handle)
{

	if (0 > libusb_control_transfer(
			device_handle,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE , // bRequestType
			0, // bRequestm
			0, // wValue
			0, // wIndex
			udi_vendor_buf_out, // pointer to destination buffer
			sizeof(udi_vendor_buf_out), // wLength
			1000 // timeout ms
			)) {
		return -1;
	}

	if (0 > libusb_control_transfer(
			device_handle,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN, // bRequestType
			0, // bRequest
			0, // wValue
			0, // wIndex
			udi_vendor_buf_in, // pointer to destination buffer
			sizeof(udi_vendor_buf_in), // wLength
			1000 // timeout ms
			)) {
		return -1;
	}

	return 0;
}

static int loop_back_interrupt(libusb_device_handle *device_handle)
{
	int transferred;
	if (0> libusb_interrupt_transfer( device_handle,
			udi_vendor_ep_interrupt_out,
			udi_vendor_buf_out,
			sizeof(udi_vendor_buf_out),
			&transferred,
			1000)) {
		return -1;
	}
	if (0> libusb_interrupt_transfer( device_handle,
			udi_vendor_ep_interrupt_in,
			udi_vendor_buf_in,
			sizeof(udi_vendor_buf_in),
			&transferred,
			1000)) {
		return -1;
	}
	return 0;
}

static int loop_back_bulk(libusb_device_handle *device_handle)
{
	int transferred;
	if (0> libusb_bulk_transfer( device_handle,
			udi_vendor_ep_bulk_out,
			udi_vendor_buf_out,
			sizeof(udi_vendor_buf_out),
			&transferred,
			1000)) {
		return -1;
	}
	if (0> libusb_bulk_transfer( device_handle,
			udi_vendor_ep_bulk_in,
			udi_vendor_buf_in,
			sizeof(udi_vendor_buf_in),
			&transferred,
			1000)) {
		return -1;
	}
	return 0;
}

static unsigned short udi_vendor_ep_iso_size;

int main() {
	struct libusb_bos_descriptor *bos_desc;
        struct libusb_config_descriptor *conf_desc;
        const struct libusb_endpoint_descriptor *endpoint;
        struct libusb_endpoint_descriptor *endpoints;
	struct libusb_device_descriptor dev_desc;
	libusb_device *dev;
	libusb_device_handle *dev_handle; //a device handle
	libusb_context *ctx = NULL; //a libusb session
	unsigned char nb_ep = 0;
	uint8_t string_index[3];
        int i, j, k, r;
        int iface, nb_ifaces, first_iface = -1;
	uint8_t endpoint_in = 0, endpoint_out = 0;      // default IN and OUT endpoints

	printf("-------------------------------\n");

	printf("Initialization library \n");
	r = libusb_init(&ctx); //initialize the library for the session we just declared
	if(r < 0) {
		perror("Init Error\n"); //there was an error
		return 1;
	}

	libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_DEBUG); //set verbosity level to 3, as suggested in the documentation

	dev_handle = libusb_open_device_with_vid_pid(ctx, VID, PID); //these are vendorID and productID I found for my usb device
	if(dev_handle == NULL)
		perror("Cannot open device\n");
	else
		printf("Device Opened\n");

	dev = libusb_get_device(dev_handle);

	printf("USB Device of ASF vendor class example found:\n");
       	printf("\nReading device descriptor:\n");
        CALL_CHECK(libusb_get_device_descriptor(dev, &dev_desc));
        printf("            length: %d\n", dev_desc.bLength);
        printf("      device class: %d\n", dev_desc.bDeviceClass);
        printf("               S/N: %d\n", dev_desc.iSerialNumber);
        printf("           VID:PID: %04X:%04X\n", dev_desc.idVendor, dev_desc.idProduct);
        printf("         bcdDevice: %04X\n", dev_desc.bcdDevice);
        printf("   iMan:iProd:iSer: %d:%d:%d\n", dev_desc.iManufacturer, dev_desc.iProduct, dev_desc.iSerialNumber);
        printf("          nb confs: %d\n", dev_desc.bNumConfigurations);
        // Copy the string descriptors for easier parsing
        string_index[0] = dev_desc.iManufacturer;
        string_index[1] = dev_desc.iProduct;
        string_index[2] = dev_desc.iSerialNumber;	

	printf("\nReading BOS descriptor: ");
        if (libusb_get_bos_descriptor(dev_handle, &bos_desc) == LIBUSB_SUCCESS) {
                printf("%d caps\n", bos_desc->bNumDeviceCaps);
                for (i = 0; i < bos_desc->bNumDeviceCaps; i++)
                        //print_device_cap(bos_desc->dev_capability[i]);
                libusb_free_bos_descriptor(bos_desc);
        } else {
                printf("no descriptor\n");
        }

	printf("\nReading first configuration descriptor:\n");
        CALL_CHECK(libusb_get_config_descriptor(dev, 0, &conf_desc));
        nb_ifaces = conf_desc->bNumInterfaces;
        printf("             nb interfaces: %d\n", nb_ifaces);
        if (nb_ifaces > 0)
                first_iface = conf_desc->interface[0].altsetting[0].bInterfaceNumber;
        for (i=0; i<nb_ifaces; i++) {
                printf("              interface[%d]: id = %d\n", i,
                        conf_desc->interface[i].altsetting[0].bInterfaceNumber);
                for (j=0; j<conf_desc->interface[i].num_altsetting; j++) {
                        printf("interface[%d].altsetting[%d]: num endpoints = %d\n",
                                i, j, conf_desc->interface[i].altsetting[j].bNumEndpoints);
                        printf("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
                                conf_desc->interface[i].altsetting[j].bInterfaceClass,
                                conf_desc->interface[i].altsetting[j].bInterfaceSubClass,
                                conf_desc->interface[i].altsetting[j].bInterfaceProtocol);
                        if ( (conf_desc->interface[i].altsetting[j].bInterfaceClass == LIBUSB_CLASS_MASS_STORAGE)
                          && ( (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x01)
                          || (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x06) )
                          && (conf_desc->interface[i].altsetting[j].bInterfaceProtocol == 0x50) ) {
                                // Mass storage devices that can use basic SCSI commands
                                //test_mode = USE_SCSI;
                        }
                        for (k=0; k<conf_desc->interface[i].altsetting[j].bNumEndpoints; k++) {
                                struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
                                endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
                                printf("       endpoint[%d].address: %02X\n", k, endpoint->bEndpointAddress);
                                // Use the first interrupt or bulk IN/OUT endpoints as default for testing
                                if ((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
                                        if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                                                if (!endpoint_in)
                                                        endpoint_in = endpoint->bEndpointAddress;
                                        } else {
                                                if (!endpoint_out)
                                                        endpoint_out = endpoint->bEndpointAddress;
                                        }
                                }
                                printf("           max packet size: %04X\n", endpoint->wMaxPacketSize);
                                printf("          polling interval: %02X\n", endpoint->bInterval);
                                libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
                                if (ep_comp) {
                                        printf("                 max burst: %02X   (USB 3.0)\n", ep_comp->bMaxBurst);
                                        printf("        bytes per interval: %04X (USB 3.0)\n", ep_comp->wBytesPerInterval);
                                        libusb_free_ss_endpoint_companion_descriptor(ep_comp);
                                }
                        }
                }
        }

	printf("- Endpoint list:\n");
	udi_vendor_ep_interrupt_in = 0;
	udi_vendor_ep_interrupt_out = 0;
	udi_vendor_ep_bulk_in = 0;
	udi_vendor_ep_bulk_out = 0;
	udi_vendor_ep_iso_in = 0;
	udi_vendor_ep_iso_out = 0;

	if(1 == conf_desc->interface->num_altsetting) {
		nb_ep = conf_desc->interface->altsetting[0].bNumEndpoints;
		endpoints = conf_desc->interface->altsetting[0].endpoint;
	} else {
		nb_ep = conf_desc->interface->altsetting[1].bNumEndpoints;
		endpoints = conf_desc->interface->altsetting[1].endpoint;
	}

	while (nb_ep) {
		nb_ep--;

		unsigned char ep_type = endpoints[nb_ep].bmAttributes
				& LIBUSB_TRANSFER_TYPE_MASK;
		unsigned char ep_add = endpoints[nb_ep].bEndpointAddress;
		unsigned char dir_in = ep_add & LIBUSB_ENDPOINT_IN;
		unsigned short ep_size = endpoints[nb_ep].wMaxPacketSize;

		switch (ep_type) {
			case LIBUSB_TRANSFER_TYPE_INTERRUPT:
				if (dir_in) {
					udi_vendor_ep_interrupt_in = ep_add;
				} else {
					udi_vendor_ep_interrupt_out= ep_add;
				}
				break;
			case LIBUSB_TRANSFER_TYPE_BULK:
				if (dir_in) {
					udi_vendor_ep_bulk_in = ep_add;
				} else {
					udi_vendor_ep_bulk_out= ep_add;
				}
				break;
			case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
				udi_vendor_ep_iso_size = ep_size;
				if (dir_in) {
					udi_vendor_ep_iso_in = ep_add;
				} else {
					udi_vendor_ep_iso_out= ep_add;
				}
				break;
		}
	}

	if (udi_vendor_ep_interrupt_in)
		printf("  - Endpoint interrupt IN:  %02X\n", udi_vendor_ep_interrupt_in );
	if (udi_vendor_ep_interrupt_out)
		printf("  - Endpoint interrupt OUT: %02X\n", udi_vendor_ep_interrupt_out);
	if (udi_vendor_ep_bulk_in)
		printf("  - Endpoint bulk      IN:  %02X\n", udi_vendor_ep_bulk_in );
	if (udi_vendor_ep_bulk_out)
		printf("  - Endpoint bulk      OUT: %02X\n", udi_vendor_ep_bulk_out);
	if (udi_vendor_ep_iso_in)
		printf("  - Endpoint iso       IN:  %02X\n", udi_vendor_ep_iso_in );
	if (udi_vendor_ep_iso_out)
		printf("  - Endpoint iso       OUT: %02X\n", udi_vendor_ep_iso_out);

	// Open interface vendor
	printf("Initialization device...\n");
	if(libusb_set_configuration(dev_handle, 1) < 0) {
		printf("error: setting config 1 failed\n");
		libusb_close(dev_handle);
		return 0;
	}
	if(libusb_claim_interface(dev_handle, 0) < 0) {
		printf("error: claiming interface 0 failed\n");
		libusb_close(dev_handle);
		return 0;
	}
	if(1 != conf_desc->interface->num_altsetting) {
		if(libusb_set_interface_alt_setting(dev_handle, 0,1) < 0) {
			printf("error: set alternate 1 interface 0 failed\n");
			libusb_close(dev_handle);
			return 0;
		}
	}
	
	printf("Control enpoint loop back...\n");
	init_buffers();
	if (loop_back_control(dev_handle)) {
		printf("Error during control endpoint transfer\n");
		libusb_close(dev_handle);
		return 0;
	}
	if (cmp_buffers()) {
		libusb_close(dev_handle);
		return 0;
	}

	if (udi_vendor_ep_interrupt_in && udi_vendor_ep_interrupt_out) {
		printf("Interrupt enpoint loop back...\n");
		init_buffers();
		if (loop_back_interrupt(dev_handle)) {
			printf("Error during interrupt endpoint transfer\n");
			libusb_close(dev_handle);
			return 0;
		}
		if (cmp_buffers()) {
			libusb_close(dev_handle);
			return 0;
		}
	}

	if (udi_vendor_ep_bulk_in && udi_vendor_ep_bulk_out) {
		printf("Bulk enpoint loop back...\n");
		init_buffers();
		if (loop_back_bulk(dev_handle)) {
			printf("Error during bulk endpoint transfer\n");
			libusb_close(dev_handle);
			return 0;
		}
		if (cmp_buffers()) {
			libusb_close(dev_handle);
			return 0;
		}
	}

        libusb_free_config_descriptor(conf_desc);
	libusb_close(dev_handle); //close the device we opened
	libusb_exit(ctx); //needs to be called to end the
	printf("------ Tests completed.\n");
	printf("-------------\n");
	return 0;
}



