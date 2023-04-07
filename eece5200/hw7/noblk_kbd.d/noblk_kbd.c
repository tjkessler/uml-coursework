// EXAMPLE NONBLOCKING KBD EVENT LOOP USING
// ncurses lib in system calls in linux
#include<stdio.h>
#include<stdlib.h>
#include<ncurses.h>

int main()
{
  int c;

//init linux ncurses  
	initscr();		//init default screen
	cbreak(); 		//process one char at a time
	keypad(stdscr,TRUE);	//enable function and arrow keys
	nodelay(stdscr,TRUE); 	//no waiting for keyboard input
	noecho(); 		//no print to screen when key depressed
	
//MENU 
mvprintw(0,0,"Controls: \n");	//print at(0,0) top-upperleft
printw("================ \n");
printw("p) Play \n");
printw("h) Hold \n");
printw("s) Stop \n");
printw("q) Quit \n");

//BEGIN: EVENT-LOOP
  c = ' ';			//init c
  while( (c=getch()) != 'q')	//BEGIN: EVENT-LOOP
	{
	 if( c !=ERR) 		//detected char input
	  {
	    switch(c)
	      {
		case 'p':
			{
	     		mvprintw(0,0,"Playing audio \n");
			break;
			}
		case 'h':
			{
	     		mvprintw(0,0,"Hold audio \n");
			break;
			}
		case 's':
			{
	     		mvprintw(0,0,"Stop audio \n");
			break;
			}
		default:
			break;	
	     };
	  };
	}
//END: EVENT-LOOP
//=================
	endwin();

} //END
