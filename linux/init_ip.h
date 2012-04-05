/*
 * =====================================================================================
 *
 *       Filename:  init_ip.h
 *
 *    Description:  header for init_ip
 *
 *        Version:  1.0
 *        Created:  03/25/12 21:28:42
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Weiyi Zheng (Edward), edward.zhengwy@gmail.com
 *        Company:  Tufts University
 *
 * =====================================================================================
 */

#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>

#define MAX_MESG 1024 /* Drone command length limit */
#define MAX_BUF 10*1024
#define NAV_BUF_SIZE 2048
#define TOUT_USEC 100000 /* select: timeout usec */
#define TOUT_SEC 0
#define TRUE 1
#define FALSE 0

/* Network configuration */
#define IPTABLE 0   //use iptables to set up NAT in linux, prevent iOS hijacking

#define PT_AT 5556 //destination port of AT command
//#define PT_AT 9891
#define PT_NAV 5554 //navdate port

//only works if IPTABLE == 1
#define PT_SD 9890/* program sending port, any unused ones are ok. remember to change system() in main() too. */
//#define PT_RC 9891/* program receiving port */

/* program constant */
#define LED_SLEEP 3 //sleep duration between each LED light
