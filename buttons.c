/*
 * buttons.c
 *
 *  Created on: 29 juil. 2020
 *      Author: Marc
 */

#include "buttons.h"
#include "stdint.h"

// Compte des keydown
uint8_t keyCountG, keyCountM, keyCountD;
uint8_t totalKey;
uint8_t repeatG, repeatD, repeatM;

// Traitement des boutons dans le cas normal
//   retourne vrai si il faut faire un envoi
bool bHandleButtonNormal(bool bG, bool bM, bool bD)
{

    // Vérifie qu'un seul bouton est pressé
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

    // retourne vrai si un bouton est pressé
    return tot>0;

}
