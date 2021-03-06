/* s2ip.c
 * initialize the udp port and socket and listen
 * used drs003 at2so.c as reference
 *
 * Written by Weiyi Zheng
 * Team Vermillion, Millennium Falcon
 * 03/25/2012
 */

#include "init_ip.h"

#define	MAX(a,b)	((a)>(b)?(a):(b))

/* global flags for programs state */
int verbose = 0;
int navflag = 0; /* navdata flag */
int srlflag = 1; /* serial terminal flag */
int ledflag = 0; /* led animation */
int running = 1; /* if want to exit, set running to 0 */

// AT command with LED animation for testing
char *LA ="AT*LED=1,2,1073741824,3\r\0";

/* global data struct */
struct global_data{
	int	sd_sock;	/* program's socket */
	struct sockaddr_in	sd_addr;	/* and it's address */
	socklen_t sd_l;
	struct sockaddr_in	dcmd;	/* drone command port address PT_AT */
	struct sockaddr_in	ccmd;	/* command copy port address */
	struct sockaddr_in navp;	/* drone navdata port */

	int rc_sock;	/* receiving socket */
	struct sockaddr_in rc_addr;	/* and it's address */

	int tty;  /* tty socket */
	struct termios ttyconfig[2]; //[0] for original, [1] for modified.

	/* file descriptor set for select */
	fd_set	pd; //master descriptor
	int pdmax; //pd boundary

	/* AT command buffer */
	int	at_n; //char number in atbuf
	char atbuf[MAX_MESG];

	/*  received packet buffer */
	int rc_n;
	char rcbuf[MAX_BUF];

	/*  navdata packet buffer */
	int navbuf_n;
	char navbuf[NAV_BUF_SIZE];
} gld;

// Read in tty command (terminal)
void Readtty() {
	int nchar;
	char buf[MAX_MESG];

	if ( (nchar = read(gld.tty, buf, sizeof(buf))) <= 1)
		return;
	else if ( nchar >= MAX_MESG ) {
		/*  should not be here */
        if (verbose) {
            fprintf(stderr,"receive too many characters\n");
        }
        return;
	}
	else {
        if (srlflag == 0) {
            buf[nchar-1] = '\r';
        }
		// only add AT commands to gld.atbuf
		if ( buf[0] == 'A' && buf[1] == 'T' ) {
			buf[nchar] = '\0';
			bcopy(buf,gld.atbuf,nchar+1);
			gld.at_n += nchar;
			return;
		}
		// exit command
        if ( buf[0]=='E' && buf[1] == 'X' && buf[2] == 'I'  && buf[3] == 'T' ) {
            running = 0;
			return;
        }
		if ( buf[0]=='.' && buf[1] == '/' && buf[2] == 's'  && buf[3] == '2' ) {
            write(gld.tty, "\002",1); /* send start signal */
			return;
        }
	}
}

/* Receive packets from PT_SD socket
 * and store in rcbuf
 */
void Readrc() {
	int nchar;
	char buf[MAX_BUF];
	struct sockaddr_in recv_addr;
	socklen_t recv_addrlen;
	int sp;

	recv_addrlen = sizeof(recv_addr);
	memset(&recv_addr, 0, sizeof(recv_addr));

	nchar = recvfrom(gld.sd_sock, buf, MAX_BUF, 0, (struct sockaddr *) &recv_addr, &recv_addrlen);

	if (nchar <= 0)
		return;
	else if ( nchar >= MAX_BUF )
		/* packet too large for buffer*/
		return;
	else {
		sp = htons(recv_addr.sin_port);
		switch (sp) {
			case 5554:
				/* navdata */
				buf[nchar] = '\0';
				bcopy(buf, gld.rcbuf, nchar+1);
				gld.rc_n += nchar;
				break;
			case 5555:
				/* video data */
				break;
			default:
				break;
		}
	}
	//fprintf(stderr,"Readrd():buff reads %d :%s\n", nchar, buf);
}

/* send AT command from atbuf to PT_AT */
void SendAT() {
	int strn_len = strnlen(gld.atbuf, MAX_MESG);
    if (verbose) { 
        fprintf(stderr,"SendAT(): strnlen %d\r\n", strn_len);
        fprintf(stderr,"%s  \r\n",gld.atbuf);
    }
	sendto(gld.sd_sock, gld.atbuf, strn_len, 0, (struct sockaddr *)&(gld.dcmd), sizeof(gld.dcmd)); 
	bzero(&gld.atbuf, sizeof(&gld.atbuf));
	gld.at_n = 0;
}

/* send packet to serial/tty (terminal) */
void Sendrc() {
	int strn_len = strnlen(gld.rcbuf, MAX_BUF);

	if ( write(gld.tty, gld.rcbuf, strn_len) <= 0 ) {
		perror("send rc error");
	}
	bzero(&gld.rcbuf, sizeof(gld.rcbuf));
	gld.rc_n = 0;
}

/* send raw nav data stream to serial/tty */
// string header is curretly defined as "NAV[space]"
void SendNav() {
    //need to implement handshake on arduino
	int strn_len = gld.navbuf_n; 
	if ( write(gld.tty, "NAV ", 4) <= 0 ) {
		perror("send rc error");
	}
	if ( write(gld.tty, gld.navbuf, strn_len) <= 0 ) {
		perror("send rc error");
	}
	bzero(&gld.navbuf, sizeof(gld.navbuf));
	gld.navbuf_n = 0;
}

/* start up nav data packet output on Drone 
 * by sending 1 to PT_NAV
 * binary of the packet content: 00 00 00 01
 */
void init_nav() {
	//make an iptables rule to reroute the NAV packet to localhost
	system("iptables -t nat -A OUTPUT -p UDP --sport 5554 -j DNAT --to 127.0.0.1:9890");
	int navsignal = 1;
	sendto(gld.sd_sock, (char*)&navsignal, sizeof(int), 0, (struct sockaddr *)&gld.navp, sizeof(gld.navp));
}

/* read in Nav data from PT_SD
 * stores it in the navbuf
 */
void ReadNav() {
	int nchar;
	char buf[NAV_BUF_SIZE];
	struct sockaddr_in recv_addr;
	socklen_t recv_addrlen;
    int sp;

	recv_addrlen = sizeof(recv_addr);
	memset(&recv_addr, 0, sizeof(recv_addr));

	nchar = recvfrom(gld.sd_sock, buf, MAX_BUF, 0, (struct sockaddr *) &recv_addr, &recv_addrlen);

	if (nchar <= 0)
		return;
	else if ( nchar > NAV_BUF_SIZE )
		/* packet too large for buffer*/
		return;
	else {
		sp = htons(recv_addr.sin_port);
		if (verbose) {
			fprintf(stderr,"ReadNAV():got packet of character %d from %d\n", nchar, sp);
		}
		switch (sp) {
			case 5554:
				/* navdata */
				bcopy(buf, gld.navbuf, nchar);
				gld.navbuf_n += nchar;
				break;
			case 5555:
				/* video data */
				break;
			default:
				break;
		}
	}
    if (verbose) {
        fprintf(stderr,"ReadNav():buff reads %d :%s\n", nchar, buf);
    }
}

/* handler for Ctrl C */
void sig_handler(int sig) {
    if (verbose) {
        fprintf(stderr,"SIGINT caught\n");
    }
    exit(0);
}

/* cleanup function upon exit. close all opened sockets */
void cleanup(void)
{
	system("iptables -F -t nat");
	int	s;
	if ((s = gld.tty) > 0) {
		gld.tty = -1;
        if (srlflag) {
            tcsetattr(s,TCSADRAIN,&(gld.ttyconfig[0]));
        }
		close(s);
	}
	if ((s = gld.sd_sock) > 0) gld.sd_sock = -1, close(s);
	//if ((s = gld.rc_sock) > 0) gld.rc_sock = -1, close(s);
}

int main(int argc, char **argv) {
    bzero(&gld,sizeof(gld));

    /* get arguments */
    int opt;
	fprintf(stderr,"program option: %s [-n] [-v] [-t] [-l]\n", argv[0]);
	fprintf(stderr,"[-n] output navdata [-v] verbose [-t] non serial input [-l] flash LED light\n");
    while ((opt = getopt(argc, argv, "nvtl")) != -1) {
        switch (opt) {
            case 'v':
                verbose = 1;
                fprintf(stderr, "verbose on\n");
                break;
            case 'n' :
                navflag = 1;
                break;
            case 't' :
                srlflag = 0;
                break;
            case 'l' :
                ledflag = 1;
                break;
            default:
               break;
        }
    }

	/* default command port on drone */
	gld.dcmd.sin_family = AF_INET;
	gld.dcmd.sin_addr.s_addr = htonl(0x7f000001); /* 127.0.0.1 localhost */
	gld.dcmd.sin_port = htons(PT_AT); //AT command port

	/* default navdata port on drone */
	gld.navp.sin_family = AF_INET;
	gld.navp.sin_addr.s_addr = htonl(0x7f000001); /* 127.0.0.1 localhost */
	gld.navp.sin_port = htons(PT_NAV); //AT command port

    if (verbose) {
        fprintf(stderr,"program running\n");
    }

	/* tty */
	/* using ICANON without ICRNL, -> read returns when '\n' is received */
	if ((gld.tty = open("/dev/tty", O_RDWR)) < 0) {
		fprintf(stderr,"tty error");
		return errno;
	}
    if (srlflag) {
        if (tcgetattr(gld.tty, &gld.ttyconfig[0])) return errno;
        gld.ttyconfig[1] = gld.ttyconfig[0];
        gld.ttyconfig[1].c_iflag = IGNBRK|IGNPAR|ISTRIP;
        gld.ttyconfig[1].c_lflag = ICANON; 	
        gld.ttyconfig[1].c_oflag = 0;
        if (tcsetattr(gld.tty,TCSANOW,&gld.ttyconfig[1])) return errno;
    }
    FD_SET(gld.tty,&gld.pd);
    gld.pdmax = MAX(gld.pdmax,gld.tty+1);

    /* program's sending socket */
	gld.sd_addr.sin_family = AF_INET;
	gld.sd_addr.sin_port = htons(PT_SD);
	if ((gld.sd_sock = socket(gld.sd_addr.sin_family, SOCK_DGRAM,0)) < 0) 
	{
		fprintf(stderr,"error creating SD socket\n");
		return errno;
	}	
	if ( bind(gld.sd_sock, (struct sockaddr *)&gld.sd_addr,sizeof(gld.sd_addr)) < 0) {
		fprintf(stderr,"error binding SD socket\n");
		return errno;
	}
	FD_SET(gld.sd_sock,&gld.pd);
	gld.pdmax = MAX(gld.pdmax,gld.sd_sock+1);

	//function upon exit
	atexit(cleanup);
    struct sigaction sa;
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) == -1 ) 
        perror("sigaction error");

	/* iptables configuration */
	if (IPTABLE) {
		/* prevent iDev highjacking of drone command port */
		system("iptables -F");						/* flush all tables */
		system("iptables -P INPUT DROP");				/* drop incoming */
		system("iptables -A INPUT -s 127.0.0.1 -j ACCEPT");		/* let local in */
		system("iptables -A INPUT -p udp --dport 67 -j ACCEPT");	/* drone DHCP server */
		system("iptables -A INPUT -p tcp --dport 5551 -j ACCEPT");	/* drone ftp (version retrival) */
		system("iptables -A INPUT -p udp --dport 5554 -j ACCEPT");	/* drone navdata port */
		system("iptables -A INPUT -p udp --dport 5555 -j ACCEPT");	/* drone video port */
		system("iptables -A INPUT -p tcp --dport 21 -j ACCEPT");	/* telnet */
		system("iptables -A INPUT -p tcp --dport 23 -j ACCEPT");	/* standard ftp */
		/* allow access to this progams port if it is not auto assigned */
		if (ntohs(gld.sd_addr.sin_port) == PT_SD)
		{
			system("iptables -A INPUT -p udp --dport 9890 -j ACCEPT");
			system("iptables -A INPUT -p udp --dport 9891 -j ACCEPT");
		}
	}

	/* navdata initialization */
	if (navflag) {
		init_nav();
	}

    //program up and running
    struct timeval tv;
	fd_set readfd;	//read descriptor
	int n;
    int strn_len; 

    tv.tv_sec = 0;
    tv.tv_usec = 300000;
	select(0,0,0,0,&tv);	/* let Arduino digest commandline echo */
	/*choice of "\002" is arbitrary */
    write(gld.tty, "\002",1); /* send start signal to arduino */
    running = 1;

    while(running) {
		/* LED animation */
        if (ledflag) {
            strn_len = strlen(LA);
            /*  sending out packets */
            if ( (sendto(gld.sd_sock, LA, strn_len, 0, (struct sockaddr *)&(gld.dcmd), sizeof(gld.dcmd)) < 0)) {
                fprintf(stderr, "sendto error\n");
                return;
            }
        }

		/* read in packets */
		readfd = gld.pd;
		tv.tv_usec = TOUT_USEC;
		tv.tv_sec = TOUT_SEC;
		n = select(gld.pdmax,&readfd,0,0, &tv);

		if (n < 0) {
			if( errno == EINTR) continue;
			else {
				fprintf(stderr,"error on select\n");
				break;
				}
		}
		else if (n == 0) {
			/* timeout */
		}
		else {
			if (FD_ISSET(gld.sd_sock,&readfd)) {
				/* sending socket has income */
				n--;
				ReadNav();
			}
			if (FD_ISSET(gld.tty, &readfd)) {
				/* terminal has input */
				n--;
				Readtty();
			}
		}
        if (ledflag) {
            sleep(LED_SLEEP);
        }
        if ( gld.at_n > 0 ) {
            SendAT();
        }
        if ( gld.rc_n >0 && navflag == 1 ) {
			// not used
            //Sendrc();
        }
        if (gld.navbuf_n > 0 && navflag == 1) {
            SendNav();
        }
    }
	return errno;
}
