#ifndef FREERTOS_APPLICATION_CONFIG_H
#define FREERTOS_APPLICATION_CONFIG_H

#define OTA            ( Unused )

#define Used           ( 1 )
#define Unused         ( 0 )

#define CONNECTION     ( WIFI )

#define ETHER          ( 1 )
#define WIFI           ( 0 )

#if (CONNECTION == 1)
#error "Connection type ETHER not supported"
#endif

#endif
