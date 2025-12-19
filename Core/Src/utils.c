/*
 * utils.c
 *
 *  Created on: Nov 29, 2025
 *      Author: jaso
 */


/**
 * converts a date formatted like __DATE__  i.e. Jan 15, 2025
 * into a date formatted as 01/15/25
 *
 * Note: the result buffer better be at least 9 bytes
 */
void convertDate(char *date, char *result) {
    strcpy(result, "0-/--/--");

    if (strncmp(date, "Jan", 3) == 0) {
        result[1] = '1';
    } else if (strncmp(date, "Feb", 3) == 0) {
        result[1] = '2';
    } else if (strncmp(date, "Mar", 3) == 0) {
        result[1] = '3';
    } else if (strncmp(date, "Apr", 3) == 0) {
        result[1] = '4';
    } else if (strncmp(date, "May", 3) == 0) {
        result[1] = '5';
    } else if (strncmp(date, "Jun", 3) == 0) {
        result[1] = '6';
    } else if (strncmp(date, "Jul", 3) == 0) {
        result[1] = '7';
    } else if (strncmp(date, "Aug", 3) == 0) {
        result[1] = '8';
    } else if (strncmp(date, "Sep", 3) == 0) {
        result[1] = '9';
    } else if (strncmp(date, "Oct", 3) == 0) {
        result[0] = '1';
        result[1] = '0';
    } else if (strncmp(date, "Nov", 3) == 0) {
        result[0] = '1';
        result[1] = '1';
    } else if (strncmp(date, "Dec", 3) == 0) {
        result[0] = '1';
        result[1] = '2';
    } else {
        result[0] = 'X';
        result[1] = 'X';
    }

    strncpy(&result[3], &date[4], 2);
    if (result[3] == ' ')
        result[3] = '0';

    strncpy(&result[6], &date[9], 2);
}


