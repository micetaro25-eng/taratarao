/*
 * motor_seigyo.c
 *
 *  Created on: Sep 20, 2025
 *      Author: shino
 */
#include "tim.h"
#include "usart.h"
#include "motor_seigyo.h"
#include "Move_sinsura.h"
#include "stm32f4xx_hal.h"

#include "stdio.h"
#include "math.h"
#include "PL_timer.h"
#include "PL_sensor.h"
#include "Wall_Control.h"
#include "G_hensu.h"

int ST_mode1, ST_mode2;
float Vs=0.0, Vt=0.0;
float G_duty_R=0.0, G_duty_L=0.0;

float G_V_maxst=0.0;
float G_V_maxtur1=0.0;
float G_V_maxtur2=0.0;
float G_V_comtur=0.0;
float G_A_maxst=0.0;
float G_A_comst=0.0;

int subfix_X_flag=0;
int doR_flag=0, doL_flag=0;
int fix_X_flag=0;
int sub_fix_mode=0;

void setVmax_Vcom(float V_maxst,float V_maxtur1,float V_maxtur2,float V_comtur,float A_maxst,float A_comst){
	G_V_maxst=V_maxst;
	G_V_maxtur1=V_maxtur1;
	G_V_maxtur2=V_maxtur2;
	G_V_comtur=V_comtur;
	G_A_maxst=A_maxst;
	G_A_comst=A_comst;
}

void kasoku(float Vs, float Vm, float Amax, float Xall, int st_mode1, int st_mode2){
	float vm_s=Vm-Vs;
	float t1;
	t1=(PI*vm_s)/(2*Amax);
	G_dtheta=PI/t1;

	G_timer=0;
	G_V_Motor=Vs;
	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	G_theta=0.0;
	G_Amax=Amax;

	G_fix_X=Xall;
	subfix_X_flag=1;
	ST_mode1=st_mode1;
	ST_mode2=st_mode2;
	if(G_state==4){
		G_state=4;
	}else if(G_state==2){
		G_state=2;
	}else{
		G_state=1;
	}

	while(1){
		if((((float)G_timer)/1000)>=(t1-0.001)){
			G_Amax=0.0;
			G_preV_Motor = G_V_Motor;
			G_V_Motor = Vm;
			G_dtheta = 0.0;
			G_theta = 0.0;
			break;
		}
	}
	while(1){
		if(Xall<=G_X_Motor){
			G_X_Motor=0.0;
			sumEnc_L=0.0;
			sumEnc_R=0.0;
			break;
		}
	}
	G_fix_X=0.0;
	subfix_X_flag=0.0;
	ST_mode1=st_mode2;
	ST_mode2=0;
}

void gensoku(float Vm, float Ve, float Amax, float Xall, int st_mode1, int st_mode2){
	float vm_e=Vm-Ve;
	float x2, t2;
	x2=(((Vm*Vm)-(Ve*Ve))*PI)/(4*Amax);
	t2=(PI*vm_e)/(2*Amax);

	G_V_Motor=Vm;
	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;

	G_fix_X=Xall;
	subfix_X_flag=1;
	ST_mode1=st_mode1;
	ST_mode2=st_mode2;

	while(1){
		if(Xall-x2<=G_X_Motor){
			break;
		}
	}

	G_timer=0;
	G_Amax=Amax;
	G_dtheta=-PI/t2;
	while(1){
		if((((float)G_timer)/1000)>=(t2-0.001)){
			G_Amax=0.0;
			G_preV_Motor = G_V_Motor;
			G_V_Motor=Ve;
			G_X_Motor=0.0;
			sumEnc_L=0.0;
			sumEnc_R=0.0;
			G_dtheta=0.0;
			G_theta=0.0;
			break;
		}
	}
	G_fix_X=0.0;
	subfix_X_flag=0;
	ST_mode1=st_mode2;
	ST_mode2=0;
}

void renStr(float Xall, int st_mode1, int st_mode2){
	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;

	G_fix_X=Xall;
	subfix_X_flag=1;
	ST_mode1=st_mode1;
	ST_mode2=st_mode2;

	while(1){
		if(Xall<=G_X_Motor){
			G_X_Motor=0.0;
			sumEnc_L=0.0;
			sumEnc_R=0.0;
			break;
		}
	}
	G_fix_X=0.0;
	subfix_X_flag=0;
	ST_mode1=st_mode2;
	ST_mode2=0;
}

//分割加速の形を決める．速度をdvだけ変える区間を，Amax/a_stepずつ加速度を変えるa_step2-1ステップ，最大加速度k2ステップ，a_step2-1ステップで作る
//sinsura，sakaku_kaitenと同じ計算．区間のステップ数は2*(a_step2-1)+k2
void bun_katati(float dv, float Amax, int a_step, int *a_step2, int *k2, float *acc2){
	float acc, wv1, wv2;
	int k;
	acc=Amax/a_step;
	*a_step2=a_step;
	*acc2=acc;
	wv1=acc*0.5f*0.001f*(a_step-1)*(a_step);
	wv2=dv-(wv1*2);
	k=wv2/(Amax*0.001f);
	*k2=k;
	if((wv2!=k*(Amax*0.001f))||(k<0)){
		*a_step2=a_step+1;
		*k2=k-1;
		if(*k2<0){		//最大加速度に届かない場合
			*k2=0;
		}
		wv1=wv1+(Amax*0.001f);
		wv2=Amax*0.001f*(*k2);
		*acc2=acc*((dv-wv2)/(2*wv1));
	}
}

//it.cの加算で進む距離．加速区間t_aステップ，等速区間ncステップ，減速区間t_dステップ
//加速度の並びが左右対称なので，1区間の速度の合計は(ステップ数)*Vs+(ステップ数+1)/2*(速度の変化)になる
float bun_kyori(float Vs, float Vm, float Ve, int t_a, int nc, int t_d){
	return (Vs*(t_a-1)*0.5f+Ve*(t_d+1)*0.5f+Vm*(t_a*0.5f+nc+t_d*0.5f))*0.001f;
}

//等速区間なしで進む距離
float bun_kyori_min(float Vs, float Vm, float Ve, float Amax, int a_step){
	int a_step2, k2;
	float acc2;
	int t_a, t_d;
	bun_katati(fmaxf(Vm-Vs,1.0f), Amax, a_step, &a_step2, &k2, &acc2);
	t_a=2*(a_step2-1)+k2;
	bun_katati(fmaxf(Vm-Ve,1.0f), Amax, a_step, &a_step2, &k2, &acc2);
	t_d=2*(a_step2-1)+k2;
	return bun_kyori(Vs, Vm, Ve, t_a, 0, t_d);
}

//bun_daikeiで決めた走り方
int bk_a_step2_a, bk_k2_a, bk_a_step2_d, bk_k2_d, bk_nc;
float bk_acc2_a, bk_amax_a, bk_acc2_d, bk_amax_d, bk_vm;

//加速区間を速度dv_aぶん，減速区間をdv_dぶん変える形で作り，等速区間ncステップでぴったりXallになるように比率を掛ける
//返り値は最大加速度/Amax(Amaxを超えないなら1以下)．saveが1なら走り方をbk_に入れる
float bun_hyouka(float Vs, float Ve, float Amax, float Xall, int a_step, int nc, float dv_a, float dv_d, int save){
	int a_step2_a, k2_a, a_step2_d, k2_d, t_a, t_d;
	float acc2_a, acc2_d, vm_bun, s_a, s_d, peak_a, peak_d;
	bun_katati(dv_a, Amax, a_step, &a_step2_a, &k2_a, &acc2_a);
	bun_katati(dv_d, Amax, a_step, &a_step2_d, &k2_d, &acc2_d);
	t_a=2*(a_step2_a-1)+k2_a;
	t_d=2*(a_step2_d-1)+k2_d;
	//ステップ数を固定すると距離は最高速度の1次式になるので，ぴったりXallになる最高速度を求める
	vm_bun=((Xall/0.001f)-(Vs*(t_a-1)*0.5f)-(Ve*(t_d+1)*0.5f))/(t_a*0.5f+nc+t_d*0.5f);
	//Vs，Veはそのままで，加速区間と減速区間の加速度にそれぞれ比率を掛けて最高速度を合わせる
	s_a=(vm_bun-Vs)/dv_a;
	s_d=(vm_bun-Ve)/dv_d;
	if(k2_a>0){
		peak_a=Amax*fabsf(s_a);
	}else{
		peak_a=(a_step2_a-1)*acc2_a*fabsf(s_a);
	}
	if(k2_d>0){
		peak_d=Amax*fabsf(s_d);
	}else{
		peak_d=(a_step2_d-1)*acc2_d*fabsf(s_d);
	}
	if(save==1){
		bk_a_step2_a=a_step2_a;
		bk_k2_a=k2_a;
		bk_acc2_a=acc2_a*s_a;
		bk_amax_a=Amax*s_a;
		bk_a_step2_d=a_step2_d;
		bk_k2_d=k2_d;
		bk_acc2_d=-acc2_d*s_d;
		bk_amax_d=-Amax*s_d;
		bk_nc=nc;
	}
	bk_vm=vm_bun;
	return fmaxf(peak_a,peak_d)/Amax;
}

//走り方を決める(すべて正の値で計算)
void bun_keikaku(float vs, float vm, float ve, float amax, float xall){
	float v_lo, v_hi, v_mid, vm_a, dv_a, dv_d, r, best_r, best_vm, x_min;
	int a_step, nc0, nc, dn, i_bun, ok_flag;
	int best_a_step, best_nc;
	float best_dv_a, best_dv_d;

	v_lo=fmaxf(vs,ve);
	if(vm<v_lo){
		vm=v_lo;
	}
	best_r=1000000.0f;
	best_vm=0.0f;
	best_a_step=0;
	best_nc=0;
	best_dv_a=1.0f;
	best_dv_d=1.0f;
	ok_flag=0;
	//分割ステップ数が多い方から試し，Amaxを超えない走り方が見つかったら決める
	for(a_step=DAIKEI_A_STEP;a_step>=2;a_step--){
		if(bun_kyori_min(vs, v_lo, ve, amax, a_step)>xall){
			continue;		//速度変化0の区間の分だけでXallを超える
		}
		//等速区間なしでもXallを超える場合は，最高速度を下げる(二分探索)
		vm_a=vm;
		if(bun_kyori_min(vs, vm, ve, amax, a_step)>xall){
			v_mid=v_lo;
			v_hi=vm;
			vm_a=v_lo;
			for(i_bun=0;i_bun<20;i_bun++){
				v_mid=(vm_a+v_hi)*0.5f;
				if(bun_kyori_min(vs, v_mid, ve, amax, a_step)<=xall){
					vm_a=v_mid;
				}else{
					v_hi=v_mid;
				}
			}
		}
		//等速区間のステップ数(切り上げ)
		nc0=0;
		if(vm_a>0.0f){
			nc0=(xall-bun_kyori_min(vs, vm_a, ve, amax, a_step))/(vm_a*0.001f);
			nc0++;
		}
		//切り上げ，その前後を試し，求めた最高速度で形を作りなおしてもう一度求める
		for(dn=-1;dn<=1;dn++){
			nc=nc0+dn;
			if(nc<0){
				continue;
			}
			dv_a=fmaxf(vm_a-vs,1.0f);
			dv_d=fmaxf(vm_a-ve,1.0f);
			for(i_bun=0;i_bun<4;i_bun++){
				r=bun_hyouka(vs, ve, amax, xall, a_step, nc, dv_a, dv_d, 0);
				if((r>1.001f)&&(bk_vm<fminf(vs,ve)*0.5f)){
					r=r+1.0f;		//Amaxを超える場合，速度が半分より落ちる走り方はなるべく選ばない
				}
				if((bk_vm<=vm*1.0001f)&&(bk_vm>=0.0f)){		//入力した最高速度は超えず，逆向きにもならない
					if(r<=1.001f){
						if((ok_flag==0)||(bk_vm>best_vm)){
							ok_flag=1;
							best_r=r;
							best_vm=bk_vm;
							best_a_step=a_step;
							best_nc=nc;
							best_dv_a=dv_a;
							best_dv_d=dv_d;
						}
					}else if((ok_flag==0)&&(r<best_r)){
						best_r=r;
						best_vm=bk_vm;
						best_a_step=a_step;
						best_nc=nc;
						best_dv_a=dv_a;
						best_dv_d=dv_d;
					}
				}
				if(bk_vm<0.0f){
					break;
				}
				dv_a=fmaxf(fabsf(bk_vm-vs),1.0f);
				dv_d=fmaxf(fabsf(bk_vm-ve),1.0f);
			}
		}
		if(ok_flag==1){
			break;
		}
	}
	if((best_a_step==0)||((ok_flag==0)&&(best_r>1.5f))){
		//ぴったり合わせるとAmaxを大きく超えるので，最高速度はmax(Vs,Ve)のまま比率を掛けずに走る
		best_a_step=DAIKEI_A_STEP;
		while((best_a_step>2)&&(bun_kyori_min(vs, v_lo, ve, amax, best_a_step)>xall)){
			best_a_step--;
		}
		best_nc=0;
		best_dv_a=fmaxf(v_lo-vs,1.0f);
		best_dv_d=fmaxf(v_lo-ve,1.0f);
		x_min=bun_kyori_min(vs, v_lo, ve, amax, best_a_step);
		if((x_min>=xall)||(v_lo<=0.0f)){
			//Amaxの中でXallまでにVeまで変えきれないので，距離を伸ばす
			xall=x_min;
		}else{
			//等速区間を一番近いステップ数にする(距離の誤差は半ステップ分まで)
			best_nc=((xall-x_min)/(v_lo*0.001f))+0.5f;
			xall=x_min+best_nc*v_lo*0.001f;
		}
	}
	bun_hyouka(vs, ve, amax, xall, best_a_step, best_nc, best_dv_a, best_dv_d, 1);
}

//VsからVm，VmからVeへ分割加速で速度を変え，Xallぴったり進む(G_stateなどの設定は呼び出し側でする)
//Vs:始めの速度 Vm:最高速度 Ve:終わりの速度．後ろ向きはVmを負にする
void bun_daikei(float Vs, float Vm, float Ve, float Amax, float Xall){
	float sgn, x_mae, x_ugoki;

	//向きをそろえて正で計算する
	if(Vm>=0.0f){
		sgn=1.0f;
	}else{
		sgn=-1.0f;
	}
	G_a_Motor=0.0f;
	G_A_Motor=0.0f;
	G_V_Motor=Vs;
	//計算している間もVsで進むので，その分を引いて計算しなおす
	x_mae=0.0f;
	G_X_Motor=0.0f;
	do{
		bun_keikaku(Vs*sgn, Vm*sgn, Ve*sgn, fabsf(Amax), fabsf(Xall)-x_mae);
		x_ugoki=fabsf(G_X_Motor);
		G_X_Motor=0.0f;
		x_mae=x_mae+x_ugoki;
	}while(x_ugoki!=0.0f);

	bun_move(bk_a_step2_a, bk_k2_a, bk_acc2_a*sgn, bk_amax_a*sgn);
	G_timer=0;
	while(1){
		if(G_timer>=bk_nc){
			break;
		}
	}
	bun_move(bk_a_step2_d, bk_k2_d, bk_acc2_d*sgn, bk_amax_d*sgn);

	G_preV_Motor = G_V_Motor;
	G_V_Motor=Ve;
}

//1区間ぶん加速度を変化させる(acc2，Amaxは符号付き)．終わると加速度は0
void bun_move(int a_step2, int k2, float acc2, float Amax){
	float sub_acc;
	sub_acc=Amax-(a_step2-1)*acc2;
	G_a_Motor=acc2;
	G_timer=0;
	while(1){
		if(G_timer==a_step2-1){
			G_A_Motor=Amax;
			G_a_Motor=0.0f;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer>=k2){
			G_a_Motor=-sub_acc;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer==1){
			G_a_Motor=-acc2;
		}
		if(G_timer==a_step2-1){
			G_a_Motor=0.0f;
			G_A_Motor=0.0f;
			break;
		}
	}
}

void sakaku_daikei(float Vs, float Vm, float Ve, float Amax, float Xall, int st_mode1, int st_mode2){
	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;

	G_fix_X=Xall;
	subfix_X_flag=1;
	ST_mode1=st_mode1;
	ST_mode2=st_mode2;

	if(Vm>=0){
		if(G_state==4){
			G_state=4;
		}else if(G_state==2){
			G_state=2;
		}else{
			G_state=1;
		}
		G_fix_X=Xall;
		subfix_X_flag=1;
	}else{
		G_state=3;
	}

	bun_daikei(Vs, Vm, Ve, Amax, Xall);

	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	G_fix_X=0.0;
	subfix_X_flag=0;
	ST_mode1=st_mode2;
	ST_mode2=0;
}

//三角関数を使っていた時のsakaku_daikei
//void sakaku_daikei(float Vs, float Vm, float Ve, float Amax, float Xall, int st_mode1, int st_mode2){
//	float vm_s=Vm-Vs, vm_e=Vm-Ve;
//	float x1, t1;
//	float x2, t2;
//	x1=(((Vm*Vm)-(Vs*Vs))*PI)/(4*Amax);
//	t1=(PI*vm_s)/(2*Amax);
//	x2=(((Vm*Vm)-(Ve*Ve))*PI)/(4*Amax);
//	t2=(PI*vm_e)/(2*Amax);

//	if(Xall>x1+x2){
//		G_dtheta=PI/t1;
//	}else{
//		if(Vm>0.0){
//			Vm=sqrt(((2*Amax*Xall)*PI_)+(((Vs*Vs)+(Ve*Ve))*0.5));
//		}else{
//			Vm=-sqrt(((2*Amax*Xall)*PI_)+(((Vs*Vs)+(Ve*Ve))*0.5));
//		}
//		vm_s=Vm-Vs;
//		vm_e=Vm-Ve;
//		x1=(((Vm*Vm)-(Vs*Vs))*PI)/(4*Amax);
//		t1=(PI*vm_s)/(2*Amax);
//		x2=(((Vm*Vm)-(Ve*Ve))*PI)/(4*Amax);
//		t2=(PI*vm_e)/(2*Amax);
//		G_dtheta=PI/t1;
//	}

//	G_timer=0;
//	G_V_Motor=Vs;		//なくていい
//	G_X_Motor=0.0;
//	sumEnc_L=0.0;
//	sumEnc_R=0.0;
//	G_theta=0.0;
//	G_Amax=Amax;

//	G_fix_X=Xall;
//	subfix_X_flag=1;
//	ST_mode1=st_mode1;
//	ST_mode2=st_mode2;

//	if(Vm>=0){
//		if(G_state==4){
//			G_state=4;
//		}else if(G_state==2){
//			G_state=2;
//		}else{
//			G_state=1;
//		}
//		G_fix_X=Xall;
//		subfix_X_flag=1;
//	}else{
//		G_state=3;
//	}
//	while(1){
//		if((((float)G_timer)/1000)>=(fabs(t1)-0.001)){
//			G_Amax=0.0;
//			G_preV_Motor = G_V_Motor;
//			G_V_Motor = Vm;
//			G_dtheta = 0.0;
//			G_theta = 0.0;
//			break;
//		}
//	}
//	while(1){
//		if((fabs(G_X_Motor))>=fabs(Xall)-fabs(x2)){
//			G_Amax=Amax;
//			G_dtheta=-(PI/t2);
//			break;
//		}
//	}
//	G_timer=0;
//	while(1){
//		if((((float)G_timer)/1000)>=(fabs(t2)-0.001)){
//			G_Amax=0.0;
//			G_preV_Motor = G_V_Motor;
//			G_V_Motor=Ve;
//			G_X_Motor=0.0;
//			sumEnc_L=0.0;
//			sumEnc_R=0.0;
//			G_dtheta=0.0;
//			G_theta=0.0;
//			break;
//		}
//	}
//	G_fix_X=0.0;
//	subfix_X_flag=0;
//	ST_mode1=st_mode2;
//	ST_mode2=0;
//}

void sakaku_kaiten(float wVm, float wAmax, float wXall, int a_step){
	float wv1_sura, wv2_sura;
	float acc_sura, acc_sura2, sub_acc, wxall_sura;
	int kk2, k2_sura, k2_sura2;
	float ca1,ca2,ca3;
	int a_step2;
	int n_all;
	float ratio_sura;
	acc_sura=wAmax/a_step;
	acc_sura2=acc_sura;
	a_step2=a_step;

	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if((wv2_sura!=k2_sura*(wAmax*0.001))||(k2_sura<0)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		if(k2_sura2<0){		//最大加速度に届かない場合
			k2_sura2=0;
		}
		wv1_sura=wv1_sura+(wAmax*0.001);
		wv2_sura=wAmax*0.001*k2_sura2;
		acc_sura2=acc_sura*((wVm-wv2_sura)/(2*wv1_sura));
	}
	wxall_sura=wVm*(2*(a_step2-1)+k2_sura2)*0.001;
	ca1=1/wAmax;
	ca2=0.002*(a_step2-1)-(2*wv1_sura/wAmax);
	if(wVm>0.0){
		if(wXall-5.0<=wxall_sura){
			ca3=-(wXall-5);
			wVm=(-ca2+sqrt((ca2*ca2)-(4*ca1*ca3)))/(2*ca1);
		}
	}else{
		if(wXall+5.0>wxall_sura){
			ca3=-(wXall+5);
			wVm=(-ca2+sqrt((ca2*ca2)-(4*ca1*ca3)))/(2*ca1);
		}
	}

	a_step2=a_step;
	acc_sura2=acc_sura;
	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if((wv2_sura!=k2_sura*(wAmax*0.001))||(k2_sura<0)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		if(k2_sura2<0){		//最大加速度に届かない場合
			k2_sura2=0;
		}
		wv1_sura=wv1_sura+(wAmax*0.001);
		wv2_sura=wAmax*0.001*k2_sura2;
		acc_sura2=acc_sura*((wVm-wv2_sura)/(2*wv1_sura));
	}
	wxall_sura=wVm*(2*(a_step2-1)+k2_sura2)*0.001;

	kk2=((wXall-wxall_sura)/(wVm*0.001));
	if(kk2*(wVm*0.001)!=(wXall-wxall_sura)){
		kk2++;
	}
	if(kk2<0){		//分割加速の分だけでwXallを超える場合は等速区間なし
		kk2=0;
	}
	//it.cの加算で出る角度はwVm*総ステップ数*0.001になるので，速度と加速度全体に比率を掛けてぴったりwXallにする
	n_all=2*(a_step2-1)+k2_sura2+kk2;
	ratio_sura=wXall/(wVm*n_all*0.001);
	wVm=wVm*ratio_sura;
	wAmax=wAmax*ratio_sura;
	acc_sura2=acc_sura2*ratio_sura;
	sub_acc=wAmax-(a_step2-1)*acc_sura2;
	G_state=6;
	Change_PID();
	G_wXturn=0.0;
	G_waturn=acc_sura2;
	sumGyro=0.0;
	G_timer=0;

	while(1){
		if(G_timer==a_step2-1){
			G_wAturn=wAmax;
			G_waturn=0.0;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer>=k2_sura2){
			G_waturn=-sub_acc;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer==1){
			G_waturn=-acc_sura2;
		}
		if(G_timer==a_step2-1){
			G_waturn=0.0;
			G_wAturn=0.0;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer>=kk2){
			G_waturn=-acc_sura2;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer>=a_step2-1){
			G_waturn=0.0;
			G_wAturn=-wAmax;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer>=k2_sura2){
			G_waturn=sub_acc;
			break;
		}
	}
	G_timer=0;
	while(1){
		if(G_timer==1){
			G_waturn=acc_sura2;
		}
		if(G_timer==a_step2-1){
			G_waturn=0.0;
			G_wAturn=0.0;
			G_wVturn=0.0;
			G_wXturn=0.0;
			sumGyro=0.0;
			break;
		}
	}
	G_X_Motor=0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	reset_sensorbuff();
}

//void sakaku_kaiten(float wVs, float wVm, float wVe, float wAmax, float wXall){
//	float vm_s=wVm-wVs, vm_e=wVm-wVe;
//	float wx1, t1;
//	float wx2,t2;
//
//	wx1=(((wVm*wVm)-(wVs*wVs))*PI)/(4*wAmax);
//	t1=(PI*vm_s)/(2*wAmax);
//	wx2=(((wVm*wVm)-(wVe*wVe))*PI)/(4*wAmax);
//	t2=(PI*vm_e)/(2*wAmax);
//
//	if(wXall>wx1+wx2){
//		G_dtheta=PI/t1;
//	}else{
//		if(wVm>0.0){
//			wVm=sqrt(((2*wAmax*wXall)*PI_)+(((wVs*wVs)+(wVe*wVe))*0.5));
//		}else{
//			wVm=-sqrt(((2*wAmax*wXall)*PI_)+(((wVs*wVs)+(wVe*wVe))*0.5));
//		}
//		vm_s=wVm-wVs;
//		vm_e=wVm-wVe;
//		wx1=(((wVm*wVm)-(wVs*wVs))*PI)/(4*wAmax);
//		t1=(PI*vm_s)/(2*wAmax);
//		wx2=(((wVm*wVm)-(wVe*wVe))*PI)/(4*wAmax);
//		t2=(PI*vm_e)/(2*wAmax);
//		G_dtheta=PI/t1;
//	}
//
//	G_timer=0;
//	G_wXturn=0.0;
//	sumEnc_L=0.0;
//	sumEnc_R=0.0;
//	sumGyro=0.0;
//	G_theta=0.0;
//	G_wAmax=wAmax;
//
//	G_state=6;
//
//	while(1){
//		if((((float)G_timer)/1000)>=(fabs(t1)-0.001)){
//			G_wAmax=0.0;
//			G_prewVturn = G_wVturn;
//			G_wVturn = wVm;
//			G_dtheta = 0.0;
//			G_theta = 0.0;
//			break;
//		}
//	}
//	while(1){
//		if((fabs(G_wXturn))>=fabs(wXall)-fabs(wx2)){
//			G_wAmax=wAmax;
//			G_dtheta=-(PI/t2);
//			break;
//		}
//	}
//	G_timer=0;
//	while(1){
//		if((((float)G_timer)/1000)>=(fabs(t2)-0.001)){
//			G_wAmax=0.0;
//			G_prewVturn = G_wVturn;
//			G_wVturn=wVe;
//			G_wXturn=0.0;
//			G_dtheta=0.0;
//			G_theta=0.0;
//			sumGyro=0.0;
//			break;
//		}
//	}
//}

void gensoku_stop(float Amax){
	float gen_t;
	gen_t=(PI*G_V_Motor)/(2*Amax);

	G_dtheta=-PI/gen_t;
	G_timer=0;
	G_X_Motor=0.0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	G_theta=0.0;
	G_Amax=Amax;

	while(1){
		if((((float)G_timer)/1000)>=(fabs(gen_t)-0.001)){
			G_Amax=0.0;
			G_preV_Motor = G_V_Motor;
			G_V_Motor=0.0;
			G_X_Motor=0.0;
			G_dtheta=0.0;
			G_theta=0.0;
			sumEnc_L=0.0;
			sumEnc_R=0.0;
			break;
		}
	}
}

void Fix_X(){
	int n_X_now, n_X_all;
	if(ST_mode2==1||ST_mode2==2){
		if(G_kabekireL_flag==0){
			doL_flag=1;
		}else{
			if((subfix_X_flag==1)&&(doL_flag==1)){
				doL_flag=0;
				fix_X_flag=1;
			}
		}
		if(G_kabekireR_flag==0){
			doR_flag=1;
		}else{
			if((subfix_X_flag==1)&&(doR_flag==1)){
				doR_flag=0;
				fix_X_flag=1;
			}
		}
	}

	if(fix_X_flag==1){
		fix_X_flag=0;
		if(ST_mode1==1){
			n_X_all=((int)G_fix_X)/G_kukaku;
			n_X_now=((int)(G_X_Motor+tohasira+(G_kukaku*0.5)))/G_kukaku;
		}else if(ST_mode1==2){
			n_X_all=((int)(G_fix_X+(G_kukaku*0.5)))/G_kukaku;
			n_X_now=((int)(G_X_Motor+tohasira+G_kukaku))/G_kukaku;
		}else if(ST_mode1==3){
			n_X_all=((int)G_fix_X)/((int)(G_kukaku*0.5*1.4141));
			n_X_now=((int)(G_X_Motor+tohasira_na+(G_kukaku*0.25*1.4141)))/((int)(G_kukaku*0.5*1.4141));
		}else if(ST_mode1==4){
			n_X_all=((int)(G_fix_X-hazimari+G_kukaku))/G_kukaku;
			n_X_now=((int)(G_X_Motor+tohasira-hazimari+(G_kukaku*1.5)))/G_kukaku;
		}else if(ST_mode1==6){
			n_X_all=((int)(G_fix_X+G_tansakumae))/G_kukaku;
			n_X_now=((int)(G_X_Motor+tohasira+G_tansakumae+(G_kukaku*0.5)))/G_kukaku;
		}

		if(ST_mode1==1){
			if(((((int)((G_X_Motor+tohasira-(G_kukaku*0.5)))%G_kukaku)+(G_kukaku*0.5))>(G_kukaku*0.75))
				&&((((int)((G_X_Motor+tohasira-(G_kukaku*0.5)))%G_kukaku)+(G_kukaku*0.5))<(G_kukaku*1.25))){
				if(ST_mode2==1){
					G_X_Motor=G_fix_X-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}else if(ST_mode2==2){
					G_X_Motor=G_fix_X-(G_kukaku*0.5)-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}
				sumEnc_L=G_X_Motor;
				sumEnc_R=G_X_Motor;
			}
		}else if(ST_mode1==2){
			if((((((int)(G_X_Motor+tohasira))%G_kukaku)+(G_kukaku*0.5))>(G_kukaku*0.75))
				&&(((((int)(G_X_Motor+tohasira))%G_kukaku)+(G_kukaku*0.5))<(G_kukaku*1.25))){
				if(ST_mode2==1){
					G_X_Motor=G_fix_X-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}else if(ST_mode2==2){
					G_X_Motor=G_fix_X-(G_kukaku*0.5)-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}
				sumEnc_L=G_X_Motor;
				sumEnc_R=G_X_Motor;
			}
		}else if(ST_mode1==3){
			if(((((int)(G_X_Motor+tohasira_na-(G_kukaku*0.25*1.4141)))%((int)(G_kukaku*0.5*1.4141))+(G_kukaku*0.25*1.4141))>(G_kukaku*0.37*1.4141))
				&&((((int)(G_X_Motor+tohasira_na-(G_kukaku*0.25*1.4141)))%((int)(G_kukaku*0.5*1.4141))+(G_kukaku*0.25*1.4141))<(G_kukaku*0.62*1.4141))){
				G_X_Motor=G_fix_X-tohasira_na-((G_kukaku*0.5*1.4141)*((float)n_X_all-(float)n_X_now));
				sumEnc_L=G_X_Motor;
				sumEnc_R=G_X_Motor;
			}
		}else if(ST_mode1==4){
			if((((int)(G_X_Motor+tohasira+G_kukaku-hazimari-(G_kukaku*0.5)))%G_kukaku+(G_kukaku*0.5)>(G_kukaku*0.75))
				&&(((int)(G_X_Motor+tohasira+G_kukaku-hazimari-(G_kukaku*0.5)))%G_kukaku+(G_kukaku*0.5)<(G_kukaku*1.25))){
				if(ST_mode2==1){
					G_X_Motor=G_fix_X-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}else if(ST_mode2==2){
					G_X_Motor=G_fix_X-(G_kukaku*0.5)-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}
				sumEnc_L=G_X_Motor;
				sumEnc_R=G_X_Motor;
			}
		}else if(ST_mode1==6){
			if((((int)(G_X_Motor+G_tansakumae+tohasira-(G_kukaku*0.5)))%G_kukaku+(G_kukaku*0.5)>(G_kukaku*0.75))
				&&(((int)(G_X_Motor+G_tansakumae+tohasira-(G_kukaku*0.5)))%G_kukaku+(G_kukaku*0.5)<(G_kukaku*1.25))){
				if(ST_mode2==1){
					G_X_Motor=G_fix_X-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}else if(ST_mode2==2){
					G_X_Motor=G_fix_X-(G_kukaku*0.5)-tohasira-(G_kukaku*((float)n_X_all-(float)n_X_now));
				}
				sumEnc_L=G_X_Motor;
				sumEnc_R=G_X_Motor;
			}
		}
	}
}

void set_CENTOR_sensor(){
	CENTER_L=g_sensor[1][0];
	CENTER_R=g_sensor[2][0];
}

void calWallControl(int modePID){
	float SENSOR_GAIN;
	int sensor_L,sensor_R;
	SENSOR_GAIN=G_V_Motor*G_P_wall;
	if(modePID==1){
		sensor_L=g_sensor[1][5];
		sensor_R=g_sensor[2][5];
		if(sensor_L>=2*CENTER_L){
			sensor_L=2*CENTER_L;
		}
		if(sensor_R>=2*CENTER_R){
			sensor_R=2*CENTER_R;
		}
		if((G_kabeL_flag==0||G_kabekireL_flag==1)&&(G_kabeR_flag==0||G_kabekireR_flag==1)){
			G_PID_walltur=0.0;
			G_state=2;
		}else if((G_kabeR_flag==0||G_kabekireR_flag==1)){
			G_PID_walltur=SENSOR_GAIN*(2*(sensor_L-CENTER_L));
		}else if((G_kabeL_flag==0||G_kabekireL_flag==1)){
			G_PID_walltur=SENSOR_GAIN*(-2*(sensor_R-CENTER_R));
		}else{
			G_PID_walltur=SENSOR_GAIN*((sensor_L-CENTER_L)-(sensor_R-CENTER_R));
		}
	}else if(modePID==2){
		if(G_kabeL_flag==1||G_kabeR_flag==1){
			G_state=1;
		}
	}else if(modePID==4){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		sensor_L=g_sensor[0][0];
		sensor_R=g_sensor[3][0];
		if(sensor_L>=(2*CENTER_naFL)){
			sensor_L=2*CENTER_naFL;
		}
		if(sensor_R>=(2*CENTER_naFR)){
			sensor_R=2*CENTER_naFR;
		}
		if(G_kabeFL_flag==1){
			G_PID_walltur=SENSOR_GAIN*sensor_L;
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		}else if(G_kabeFR_flag==1){
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			G_PID_walltur=-SENSOR_GAIN*sensor_R;
		}else{
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			G_PID_walltur=0.0;
		}
	}else if(modePID>=10){
		if(G_kabeFL_flag==1&&G_kabeFR_flag==1){
			SENSOR_GAIN=G_P_wall*((float)(1000)/(float)(CENTER_FL));
			sensor_L=g_sensor[0][0];
			sensor_R=g_sensor[3][0];
			if(sensor_L>=(2*CENTER_FL)){
				sensor_L=2*CENTER_FL;
			}
			if(sensor_R>=(2*CENTER_FR)){
				sensor_R=2*CENTER_FR;
			}
			G_PID_wallstr=-SENSOR_GAIN*((sensor_L-CENTER_FL)+(sensor_R-CENTER_FR));
			if(modePID==10){
				G_PID_walltur=-4.0*SENSOR_GAIN*((sensor_L-CENTER_FL)-(sensor_R-CENTER_FR));
				G_PID_wallstr=-2.0*SENSOR_GAIN*((sensor_L-CENTER_FL)+(sensor_R-CENTER_FR));
			}else if(modePID==11){
				G_PID_walltur=-6.0*SENSOR_GAIN*((sensor_L-CENTER_FL)-(sensor_R-CENTER_FR));
				G_PID_wallstr=-3.0*SENSOR_GAIN*((sensor_L-CENTER_FL)+(sensor_R-CENTER_FR));
			}else if(modePID==12){
				G_PID_walltur=-10.0*SENSOR_GAIN*((sensor_L-CENTER_FL)-(sensor_R-CENTER_FR));
				G_PID_wallstr=-5.0*SENSOR_GAIN*((sensor_L-CENTER_FL)+(sensor_R-CENTER_FR));
			}
		}
	}else{
		G_PID_walltur=0.0;
		G_PID_wallstr=0.0;
	}
}

void judge_PID(){
	if((G_state==1)&&(g_sensor[0][0]+g_sensor[3][0]>=sekuPIDwall_F)){
		G_doPID_flag=0;
	}else if(G_stopPID_flag==1&&G_state<10){
		G_doPID_flag=0;
	}else if(G_state>=10){
		G_doPID_flag=1;
	}else{
		if(ST_mode1==0){
			G_doPID_flag=0;
		}else if(ST_mode1==1){			//0,壁制御しない，1区画，2半区画，3斜め，4始まり，5無条件，6探索
			if((((int)G_X_Motor)%G_kukaku<G_kukaku-tohasira-G_noPIDwall)
				||(((int)G_X_Motor)%G_kukaku>G_kukaku-tohasira+G_noPIDwall)){
				G_doPID_flag=1;
			}else{
				G_doPID_flag=0;
			}
		}else if(ST_mode1==2){
			if((((int)G_X_Motor)%G_kukaku<(G_kukaku*0.5)-tohasira-G_noPIDwall)
				||(((int)G_X_Motor)%G_kukaku>(G_kukaku*0.5)-tohasira+G_noPIDwall)){
				G_doPID_flag=1;
			}else{
				G_doPID_flag=0;
			}
		}else if(ST_mode1==3){
			G_doPID_flag=1;
		}else if(ST_mode1==4){
			if((((int)G_X_Motor)%G_kukaku<hazimari-tohasira-G_noPIDwall)
				||(((int)G_X_Motor)%G_kukaku>hazimari-tohasira+G_noPIDwall)){
				G_doPID_flag=1;
			}else{
				G_doPID_flag=0;
			}
		}else if(ST_mode1==5){
			G_doPID_flag=1;
		}else if(ST_mode1==6){
			if((((int)G_X_Motor)%G_kukaku<G_kukaku-tohasira-G_tansakumae-G_noPIDwall)
				||(((int)G_X_Motor)%G_kukaku>G_kukaku-tohasira-G_tansakumae+G_noPIDwall)){
				G_doPID_flag=1;
			}else{
				G_doPID_flag=0;
			}
		}
	}
}

void Duty_Dire() {
	float Enc_ave[2], sumEnc_ave;
	float hensu_sP, hensu_sI, hensu_sD;
	float hensu_tP, hensu_tI, hensu_tD;
	float hensu_svF, hensu_saF, hensu_tvF, hensu_taF;

	Enc_ave[0]=(Enc_L[0]+Enc_R[0])*0.5;
	Enc_ave[1]=(Enc_L[1]+Enc_R[1])*0.5;
	sumEnc_ave=(sumEnc_L+sumEnc_R)*0.5;

	hensu_sP = G_V_Motor - Enc_ave[0];
	hensu_sI = G_X_Motor - sumEnc_ave;
	hensu_sD = (G_V_Motor - G_preV_Motor) - (Enc_ave[0] - Enc_ave[1]);
	hensu_tP = G_wVturn - Gyro[0];
	hensu_tI = G_wXturn - sumGyro;
	hensu_tD = (G_wVturn - G_prewVturn) - (Gyro[0] - Gyro[1]);
	hensu_svF = G_V_Motor;
	hensu_saF = G_A_Motor;
	hensu_tvF = G_wVturn;
	hensu_taF = G_wAturn;


//	if (((G_wVturn >= 0) && (hensu_tP >= (G_wVturn * G_Maxrate_hensu_tP)))
//			|| ((G_wVturn < 0) && (hensu_tP <= (G_wVturn * G_Maxrate_hensu_tP)))) {
//		hensu_tP = (G_wVturn * G_Maxrate_hensu_tP);
//	} else if (((G_wVturn >= 0) && (hensu_tP <= -(G_wVturn * G_Maxrate_hensu_tP)))
//			|| ((G_wVturn < 0) && (hensu_tP >= -(G_wVturn * G_Maxrate_hensu_tP)))) {
//		hensu_tP = -(G_wVturn * G_Maxrate_hensu_tP);
//	}
//
//	if (((G_wXturn >= 0) && (hensu_tI >= (G_wXturn * G_Maxrate_hensu_tI)))
//			|| ((G_wXturn < 0) && (hensu_tI <= (G_wXturn * G_Maxrate_hensu_tI)))) {
//		hensu_tI = (G_wXturn * G_Maxrate_hensu_tI);
//	} else if (((G_wXturn >= 0) && (hensu_tI <= -(G_wXturn * G_Maxrate_hensu_tI)))
//			|| ((G_wXturn < 0) && (hensu_tI >= -(G_wXturn * G_Maxrate_hensu_tI)))) {
//		hensu_tI = -(G_wXturn * G_Maxrate_hensu_tI);
//	}

	Vs = G_Pgain_Str * hensu_sP
			+ G_Igain_Str * hensu_sI
			+ G_Dgain_Str * hensu_sD
			+ G_vFgain_Str * hensu_svF
			+ G_aFgain_Str * hensu_saF;
	Vs += G_PID_wallstr;

	Vt = G_Pgain_Tur * hensu_tP
			+ G_Igain_Tur * hensu_tI
			+ G_Dgain_Tur * hensu_tD
			+ G_vFgain_Tur * hensu_tvF
			+ G_aFgain_Tur * hensu_taF;
	Vt += G_PID_walltur;

	G_duty_L=((Vs+Vt)/g_V_batt)*100.0;
	G_duty_R=((Vs-Vt)/g_V_batt)*100.0;

	if(G_duty_L>0.0){
		HAL_GPIO_WritePin(MotorIN1_L_GPIO_Port, MotorIN1_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MotorIN2_L_GPIO_Port, MotorIN2_L_Pin, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(MotorIN1_L_GPIO_Port, MotorIN1_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MotorIN2_L_GPIO_Port, MotorIN2_L_Pin, GPIO_PIN_SET);
	}

	if(G_duty_L<0.0){
		G_duty_L=-G_duty_L;
	}
	if(G_duty_L>100.0){
		G_duty_L=100.0;
	}

	if(G_duty_R>0.0){
		HAL_GPIO_WritePin(MotorIN1_R_GPIO_Port, MotorIN1_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MotorIN2_R_GPIO_Port, MotorIN2_R_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(MotorIN1_R_GPIO_Port, MotorIN1_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MotorIN2_R_GPIO_Port, MotorIN2_R_Pin, GPIO_PIN_RESET);
	}

	if(G_duty_R<0.0){
		G_duty_R=-G_duty_R;
	}
	if(G_duty_R>100.0){
		G_duty_R=100.0;
	}
}

void DC_Motor_move(){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)(G_duty_R));	//100でfull
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (int)(G_duty_L));	//100でfull
}

void Change_PID(){
	if(test_san_flag!=1){
		if(G_kyuin_flag==0){
			if(G_state==0){
				G_Pgain_Str=0;
				G_Igain_Str=0;
				G_Dgain_Str=0;
				G_Pgain_Tur=0;
				G_Igain_Tur=0;
				G_Dgain_Tur=0;
				G_vFgain_Str=0;
				G_aFgain_Str=0;
				G_vFgain_Tur=0;
				G_aFgain_Tur=0;
				G_P_wall=0.0;
			}else if(G_state==1){
				G_Pgain_Str = g_Pe_KbSt;
				G_Igain_Str = g_Ie_KbSt;
				G_Dgain_Str = g_De_KbSt;
				G_Pgain_Tur = g_Pj_KbSt;
				G_Igain_Tur = g_Ij_KbSt;
				G_Dgain_Tur = g_Dj_KbSt;
				G_vFgain_Str = g_FV_KbSt;
				G_aFgain_Str = g_FA_KbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = g_P_wall;
			}else if(G_state==2){
				G_Pgain_Str = g_Pe_noKbSt;
				G_Igain_Str = g_Ie_noKbSt;
				G_Dgain_Str = g_De_noKbSt;
				G_Pgain_Tur = g_Pj_noKbSt;
				G_Igain_Tur = g_Ij_noKbSt;
				G_Dgain_Tur = g_Dj_noKbSt;
				G_vFgain_Str = g_FV_noKbSt;
				G_aFgain_Str = g_FA_noKbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = 0.0;
			}else if(G_state==3){
				G_Pgain_Str = g_Pe_B;
				G_Igain_Str = 0;
				G_Dgain_Str = 0;
				G_Pgain_Tur = g_Pj_B;
				G_Igain_Tur = 0;
				G_Dgain_Tur = 0;
				G_vFgain_Str = g_FV_B;
				G_aFgain_Str = g_FA_B;
				G_vFgain_Tur = 0;
				G_aFgain_Tur = 0;
				G_P_wall = 0.0;
			}else if(G_state==4){
				G_Pgain_Str = g_Pe_noKbSt;
				G_Igain_Str = g_Ie_noKbSt;
				G_Dgain_Str = g_De_noKbSt;
				G_Pgain_Tur = g_Pj_noKbSt;
				G_Igain_Tur = g_Ij_noKbSt;
				G_Dgain_Tur = g_Dj_noKbSt;
				G_vFgain_Str = g_FV_noKbSt;
				G_aFgain_Str = g_FA_noKbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = g_P_nawall;
			}else if(G_state==6){
				G_Pgain_Str=g_Pe_Tur;
				G_Igain_Str=g_Ie_Tur;
				G_Dgain_Str=g_De_Tur;
				G_Pgain_Tur=g_Pj_Tur;
				G_Igain_Tur=g_Ij_Tur;
				G_Dgain_Tur=g_Dj_Tur;
				G_vFgain_Str=0.0;
				G_aFgain_Str=0.0;
				G_vFgain_Tur=g_FwV_Tur;
				G_aFgain_Tur=g_FwA_Tur;
				G_P_wall=0.0;
			}else if(G_state==8){
				G_Pgain_Str=g_Pe_Sur;
				G_Igain_Str=g_Ie_Sur;
				G_Dgain_Str=g_De_Sur;
				G_Pgain_Tur=g_Pj_Sur;
				G_Igain_Tur=g_Ij_Sur;
				G_Dgain_Tur=g_Dj_Sur;
				G_vFgain_Str=g_FV_Sur;
				G_aFgain_Str=g_FA_Sur;
				G_vFgain_Tur=g_FwV_Sur;
				G_aFgain_Tur=g_FwA_Sur;
				G_P_wall=0.0;
			}else if(G_state>=10){
				G_Pgain_Str=0;
				G_Igain_Str=0;
				G_Dgain_Str=0;
				G_Pgain_Tur=0;
				G_Igain_Tur=0;
				G_Dgain_Tur=0;
				G_vFgain_Str=0;
				G_aFgain_Str=0;
				G_vFgain_Tur=0;
				G_aFgain_Tur=0;
				G_P_wall=g_P_maewall;
			}
		}else{
			if(G_state==0){
				G_Pgain_Str=0;
				G_Igain_Str=0;
				G_Dgain_Str=0;
				G_Pgain_Tur=0;
				G_Igain_Tur=0;
				G_Dgain_Tur=0;
				G_vFgain_Str=0;
				G_aFgain_Str=0;
				G_vFgain_Tur=0;
				G_aFgain_Tur=0;
				G_P_wall=0.0;
			}else if(G_state==1){
				G_Pgain_Str = g_Pe_KKbSt;
				G_Igain_Str = g_Ie_KKbSt;
				G_Dgain_Str = g_De_KKbSt;
				G_Pgain_Tur = g_Pj_KKbSt;
				G_Igain_Tur = g_Ij_KKbSt;
				G_Dgain_Tur = g_Dj_KKbSt;
				G_vFgain_Str = g_FV_KKbSt;
				G_aFgain_Str = g_FA_KKbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = g_P_Kwall;
			}else if(G_state==2){
				G_Pgain_Str = g_Pe_KnoKbSt;
				G_Igain_Str = g_Ie_KnoKbSt;
				G_Dgain_Str = g_De_KnoKbSt;
				G_Pgain_Tur = g_Pj_KnoKbSt;
				G_Igain_Tur = g_Ij_KnoKbSt;
				G_Dgain_Tur = g_Dj_KnoKbSt;
				G_vFgain_Str = g_FV_KnoKbSt;
				G_aFgain_Str = g_FA_KnoKbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = 0.0;
			}else if(G_state==4){
				G_Pgain_Str = g_Pe_KnoKbSt;
				G_Igain_Str = g_Ie_KnoKbSt;
				G_Dgain_Str = g_De_KnoKbSt;
				G_Pgain_Tur = g_Pj_KnoKbSt;
				G_Igain_Tur = g_Ij_KnoKbSt;
				G_Dgain_Tur = g_Dj_KnoKbSt;
				G_vFgain_Str = g_FV_KnoKbSt;
				G_aFgain_Str = g_FA_KnoKbSt;
				G_vFgain_Tur = 0.0;
				G_aFgain_Tur = 0.0;
				G_P_wall = g_P_Knawall;
			}else if(G_state==8){
				G_Pgain_Str=g_Pe_KSur;
				G_Igain_Str=g_Ie_KSur;
				G_Dgain_Str=g_De_KSur;
				G_Pgain_Tur=g_Pj_KSur;
				G_Igain_Tur=g_Ij_KSur;
				G_Dgain_Tur=g_Dj_KSur;
				G_vFgain_Str=g_FV_KSur;
				G_aFgain_Str=g_FA_KSur;
				G_vFgain_Tur=g_FwV_KSur;
				G_aFgain_Tur=g_FwA_KSur;
				G_P_wall=0.0;
			}
		}
	}
}
