/*
 * Moveonly_sura.c
 *
 *  Created on: Oct 8, 2025
 *      Author: shino
 */

#include "moveonly_sura.h"

#include "motor_seigyo.h"
#include "move_sinsura.h"
#include "Wall_record.h"

#include "G_hensu.h"
// g_goal　saitan用のかえた？

int i_suraon;

void sura_moveR(){
	if(G_saitan_mode==0){
		sinsura(600.0 ,8000.0 ,91.0 ,G_tansakumae ,20.0, 0, 10);
	}else if(G_saitan_mode==1){
		sinsura(690.0 ,10000.0 ,97.0 ,20.0 ,20.0, 0, 10);	//最短小回り探索
	}else if(G_saitan_mode==2){
		sinsura(1300.0 ,22000.0 ,95.0 ,3.0 ,33.0, 0, 10);	//最短小回り前距離小
	}else if(G_saitan_mode==3){
		sinsura(1300.0 ,25000.0 ,89.0 ,9.0 ,3.0, 0, 5);	//吸引小回り
	}else if(G_saitan_mode==4){
		sinsura(1600.0 ,70000.0 ,105.0 ,7.0 ,70.0, 0, 10);
	}
	G_direction+=90;
}

void sura_moveL(){
	if(G_saitan_mode==0){
		sinsura(-600.0 ,-8000.0 ,-91.0 ,G_tansakumae ,20.0, 0, 10);
	}else if(G_saitan_mode==1){
		sinsura(-690.0 ,-10000.0 ,-97.0 ,20.0 ,20.0, 0, 10);	//最短小回り探索
	}else if(G_saitan_mode==2){
		sinsura(-1300.0 ,-22000.0 ,-95.0 ,3.0 ,33.0, 0, 10);	//最短小回り前距離小
	}else if(G_saitan_mode==3){
		sinsura(-1300.0 ,-25000.0 ,-89.0 ,9.0 ,3.0, 0, 5);	//吸引小回り
	}else if(G_saitan_mode==4){
		sinsura(-1600.0 ,-70000.0 ,-105.0 ,7.0 ,70.0, 0, 10);
	}

	G_direction-=90;
}

void daisura_moveR(int mode){
	if(mode==0){
		if(G_saitan_mode==1){
			sinsura(1700.0 ,26000.0 ,93.1 ,19.0 ,55.0, 1, 10);	//最短小回り探索
		}else if(G_saitan_mode==2){
			sinsura(1700.0 ,26000.0 ,93.1 ,19.0 ,55.0, 1, 10);	//最短小回り
		}else if(G_saitan_mode==3||G_saitan_mode==4){
			sinsura(1700.0 ,19000.0 ,90 ,19.0 ,58.0, 1, 10);	//最短吸引
		}
	}else if(mode==1){
		sinsura2(1500.0 ,40000.0 ,95.0 ,35.0 ,122.0, 1, G_V_maxtur2, G_A_comst);	//12
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);		//21
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);		//22
	}
	G_direction-=90;
}

//変えてる
void daisura_moveL(int mode){
	if(mode==0){
		if(G_saitan_mode==1){
			sinsura(-1700.0 ,-26000.0 ,-93.1 ,19.0 ,55.0, 1, 10);	//最短小回り探索
		}else if(G_saitan_mode==2){
			sinsura(-1700.0 ,-26000.0 ,-93.1 ,19.0 ,55.0, 1, 10);	//最短小回り
		}else if(G_saitan_mode==3||G_saitan_mode==4){
			sinsura(-1700.0 ,-19000.0 ,-90 ,19.0 ,58.0, 1, 10);	//最短吸引
		}
	}else if(mode==1){
		sinsura2(-1500.0 ,40000.0 ,95.0 ,35.0 ,122.0, 1, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(-700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(-700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction-=90;
}

void in180sura_moveR(int mode){
	if(mode==0){
		if(G_saitan_mode==1){
			sinsura(975.0 ,30000.0 ,185.2 ,30.0 ,73.0, 1, 10);		//最短小回り探索
		}else if(G_saitan_mode==2){
			sinsura(975.0 ,30000.0 ,185.2 ,30.0 ,73.0, 1, 10);		//最短小回り
		}else if(G_saitan_mode==3||G_saitan_mode==4){
			sinsura(950.0 ,16000.0 ,180.0 ,20.0 ,60.0, 1, 10);		//最短吸引
		}
	}else if(mode==1){
		sinsura2(800.0 ,50000.0 ,199.0 ,30.0 ,105.0, 1, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=180;
}

void in180sura_moveL(int mode){
	if(mode==0){
		if(G_saitan_mode==1){
			sinsura(-975.0 ,-30000.0 ,-185.2 ,30.0 ,73.0, 1, 10);		//最短小回り探索
		}else if(G_saitan_mode==2){
			sinsura(-975.0 ,-30000.0 ,-185.2 ,30.0 ,73.0, 1, 10);		//最短小回り
		}else if(G_saitan_mode==3||G_saitan_mode==4){
			sinsura(-950.0 ,-16000.0 ,-180.0 ,20.0 ,60.0, 1, 10);		//最短吸引
		}
	}else if(mode==1){
		sinsura2(-800.0 ,50000.0 ,199.0 ,30.0 ,105.0, 1, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=180;
}


void in45sura_moveR(int mode){
	if(mode==0){
		sinsura(1200.0 ,45000.0 ,45.0 ,0.0 ,76.0, 3, 10);
	}else if(mode==1){
		sinsura2(1800.0 ,95000.0 ,46.0 ,3.0 ,115.0, 3, G_V_maxtur2, G_A_comst);
	}else if(mode==2){//多分ない
		sinsura2(1800.0 ,95000.0 ,46.0 ,3.0 ,115.0, 3, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(1800.0 ,95000.0 ,46.0 ,3.0 ,115.0, 3, G_V_maxtur2, G_A_comst);
	}
	G_direction+=45;
}

void in45sura_moveL(int mode){
	if(mode==0){
		sinsura(-1200.0 ,-45000.0 ,-45.0 ,0.0 ,76.0, 3, 10);
	}else if(mode==1){
		sinsura2(-1800.0 ,95000.0 ,46.0 ,3.0 ,115.0, 3, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction-=45;
}

void out45sura_moveR(int mode){
	if(mode==0){
		sinsura(1100.0 ,25000.0 ,45.0 ,66.0 ,25.0, 2, 10);
	}else if(mode==1){
		sinsura2(1100.0 ,40000.0 ,49.0 ,35.0 ,45.0, 2, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=45;
}

void out45sura_moveL(int mode){
	if(mode==0){
		sinsura(-1100.0 ,-25000.0 ,-45.0 ,66.0 ,25.0, 2, 10);
	}else if(mode==1){
		sinsura2(-1100.0 ,40000.0 ,49.0 ,35.0 ,45.0, 2, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction-=45;
}

void in135sura_moveR(int mode){
	if(mode==0){
		sinsura(1200.0 ,27500.0 ,135.0 ,26.0 ,54.0, 3, 10);
	}else if(mode==1){
		sinsura2(1400.0 ,40000.0 ,145.0 ,35.0 ,120.0, 3, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=135;
}

void in135sura_moveL(int mode){
	if(mode==0){
		sinsura(-1200.0 ,-27500.0 ,-135.0 ,26.0 ,54.0, 3, 10);
	}else if(mode==1){
		sinsura2(-1400.0 ,40000.0 ,145.0 ,35.0 ,120.0, 3, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction-=135;
}

void out135sura_moveR(int mode){
	if(mode==0){
		sinsura(1200.0 ,32500.0 ,135.0 ,76.0 ,96.0, 2, 10);
	}else if(mode==1){
		sinsura2(1100.0 ,40000.0 ,145.0 ,51.0 ,133.0, 2, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=135;
}

void out135sura_moveL(int mode){
	if(mode==0){
		sinsura(-1200.0 ,-32500.0 ,-135.0 ,76.0 ,96.0, 2, 10);
	}else if(mode==1){
		sinsura2(-1100.0 ,40000.0 ,145.0 ,51.0 ,133.0, 2, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction-=135;
}

void v_90sura_moveR(int mode){
	if(mode==0){
		sinsura(1200.0 ,35000.0 ,90.0 ,40.0 ,37.0, 4, 10);
	}else if(mode==1){
		sinsura2(1400.0 ,60000.0 ,100.0 ,13.0 ,100.0, 4, G_V_maxtur2, G_A_comst);
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);
	}
	G_direction+=90;
}

void v_90sura_moveL(int mode){
	if(mode==0){
		sinsura(-1200.0 ,-35000.0 ,-90.0 ,40.0 ,37.0, 4, 10);	//11
	}else if(mode==1){
		sinsura2(-1400.0 ,60000.0 ,100.0 ,13.0 ,100.0, 4, G_V_maxtur2, G_A_comst);	//12
	}else if(mode==2){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur1, G_A_comst);		//21
	}else if(mode==3){
		sinsura2(700.0 ,25000.0 ,96.0 ,35.0 ,85.0, 1, G_V_maxtur2, G_A_comst);		//22
	}
	G_direction-=90;
}
