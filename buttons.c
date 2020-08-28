/*
 * buttons.c
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#include "buttons.h"
#include "stdint.h"

typedef enum {  tmOff, tmCount, tmTurnLEDoff, tmTurnLEDoffAndButtonOn,
            tmTestProg, tmTurnLEDoffTestProg,
            tmReadButtonState // pour lire l'�tat du bouton apr�s d�tection du mouvement
} tTimerMode;


// Compte des keydown
uint8_t keyCountG, keyCountM, keyCountD;
uint8_t totalKey;
uint8_t repeatG, repeatD, repeatM;

// Traitement des boutons dans le cas normal
//   retourne vrai si il faut faire un envoi
bool bHandleButtonNormal(bool bG, bool bM, bool bD)
{

    // V�rifie qu'un seul bouton est press�
    int tot = bG + bM + bD;
    if (tot == 1) {
        if (bG) {
            keyCountG++;
            totalKey++;
            repeatG=4;
        }

        if (bM) {
            keyCountM++;
            totalKey++;
            repeatM=4;
        }


        if (bD) {
            keyCountD++;
            totalKey++;
            repeatD=4;
        }
    }

    // retourne vrai si un bouton est press�
    return tot>0;

}

#if 1==0
// Le timer sert pour de petites t�ches
void bHandleTimer()
{
    // TODO copie du projet Microchip

    bool bG = PORTBbits.RB4==0;
    bool bM = PORTBbits.RB5==0;
#ifdef NO_RIGHT_BUTTON
    bool bD = false;
#else
    bool bD = PORTBbits.RB6==0;
#endif

    switch(mTimerMode) {

            // Am�ne le compteur � 0 pour g�n�rer un d�lais donn�
        case tmCount:
            if (timerCounter>0) {
                timerCounter--;
                if (timerCounter==0) {
                    mTimerMode = tmOff;
                    hwDisableTimerInterrupt();
                }
            }
            break;

        case tmReadButtonState:
            // Lit les boutons apr�s un d�lais (d�clench� par interrupt bouton)
            hwDisableTimerInterrupt();

            switch(progMode) {

                case pmNone:
                    mHandleButtonNormal(bG, bM, bD);
                    break;

                case pmPhase3: // En train de montrer le compte, si on a des boutons, on sort
                    progCheckNoButton(bG, bM, bD);
                    break;

                case pmPhase4: // attente du bouton orange pour aller � la suite
                    progCheckButton(bG, bM, bD);
                    break;

                case pmProg:
                    progHandleButton(bG, bM, bD);
                    break;

                case pmProgFreq:
                    progHandleButtonFreq(bG, bM, bD);
                    break;

                default:
                    // On devrait pas �tre l�...
                    mTempResetCause=1;
                    break;
            }
            break;

        case tmTurnLEDoff:
            hwClearLED(LEDG|LEDM|LEDD);
            hwDisableTimerInterrupt();
            break;

            // Appel� apr�s la v�rification de l'�tat bas d'un bouton
            // V�rifie que c'est le bouton G qui est press�
        case tmTurnLEDoffTestProg:
            if (bG) {
                // hwSetLED(LEDM);
                mTimerMode=tmTestProg;
            } else {
                hwClearLED(LEDG|LEDM|LEDD);
                mTimerMode=tmOff;
                hwDisableTimerInterrupt();
            }
            break;

            // V�rifie que le bouton D est press�
        case tmTestProg:
#ifdef NO_RIGHT_BUTTON
            if (bM) {
#else
            if (bD) {
#endif
                // hwSetLED(LEDD);
                progTestGoProg=true;
            }
            hwClearLED(LEDG|LEDM|LEDD);
            hwDisableTimerInterrupt();
            break;

        case tmTurnLEDoffAndButtonOn:
            hwClearLED(LEDG|LEDM|LEDD);
            hwDisableTimerInterrupt();
            hwEnableButtonsInterrupt();
            break;

        default:
            hwClearLED(LEDG|LEDM|LEDD);
            hwSetLED(LEDM|LEDD);
            hwDisableTimerInterrupt();
            hwEnableButtonsInterrupt();
            break;
    }

}
#endif
