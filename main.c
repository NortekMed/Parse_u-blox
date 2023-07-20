#include <stdio.h>
#include <math.h>
#include <dirent.h>
#include <string.h>

int main(int argc, char* argv[]) {

	DIR* d;
	struct dirent* dir;

	char* dirName = "/media/pi/usbdisk/Essai_GNSS_u-blox_Vagues/logs/";
	d = opendir(dirName);
	if (d) {
		while ((dir = readdir(d)) != NULL) {
			if (strlen(dir->d_name) <= 2){
				continue;
			}
			char* inputFile = malloc(100 * sizeof(char));
			strcpy(inputFile, dirName);
			strcat(inputFile, dir->d_name);

			char* prefix = malloc(100 * sizeof(char));
			memcpy(prefix, dir->d_name, sizeof(char) * (strlen(dir->d_name) - 4));



			printf("Input file: %s\n", dir->d_name);

			char* outputFile_vel = malloc(100 * sizeof(char));
			strcpy(outputFile_vel, "/media/pi/usbdisk/Essai_GNSS_u-blox_Vagues/velGNSS/");
			strcat(outputFile_vel, prefix);
			strcat(outputFile_vel, ".txt");
			
			char* outputFile_PVT = malloc(100 * sizeof(char));
			strcpy(outputFile_PVT, "/media/pi/usbdisk/Essai_GNSS_u-blox_Vagues/PVT/");
			strcat(outputFile_PVT, prefix);
			strcat(outputFile_PVT, ".txt");

		    // open input file
			FILE* fin = fopen(inputFile, "rb");
			if (fin == NULL) {
				printf("input file not found\n");
				return 1;
			}

			// open output file(s)
			FILE* fout1 = fopen(outputFile_vel, "w");
			if (fout1 == NULL) {
				printf("output file cannot be opened\n");
				return 1;
			}

			FILE* fout2 = fopen(outputFile_PVT, "w");
			if (argc == 4) {
				if (fout2 == NULL) {
					printf("output file cannot be opened\n");
					return 1;
				}
			}

			unsigned char c1 = 1;
			unsigned char c2 = 1;
			unsigned char c3 = 1;
			unsigned char c4 = 1;

			// parse input file
			while (1) {
				size_t ntmp = fread(&c2, sizeof(char), 1, fin);
				if (ntmp == 0) {
					break;
				}

				if (c1 == 0xb5 && c2 == 0x62) { // ubx frame found

					ntmp = fread(&c1, sizeof(char), 1, fin);
					ntmp = fread(&c2, sizeof(char), 1, fin);

					if (c1 == 0x01 && c2 == 0x12) { // ubx-nav-velned frame found

						unsigned short nfield = 0;
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						nfield = c2 << 8 | c1;

						if (nfield == 36) { // normal value for the ubx-nav-velned message

							// skip 4 bytes (GPS time of week)
							for (int u = 0; u < 4; u++) {
								ntmp = fread(&c1, sizeof(char), 1, fin);
							}
							// read North velocity
							fread(&c1, sizeof(char), 1, fin);
							fread(&c2, sizeof(char), 1, fin);
							fread(&c3, sizeof(char), 1, fin);
							fread(&c4, sizeof(char), 1, fin);
							int vNtmp = 0;
							vNtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;

							// read East velocity
							fread(&c1, sizeof(char), 1, fin);
							fread(&c2, sizeof(char), 1, fin);
							fread(&c3, sizeof(char), 1, fin);
							fread(&c4, sizeof(char), 1, fin);
							int vEtmp = 0;
							vEtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;

							// read Down velocity
							fread(&c1, sizeof(char), 1, fin);
							fread(&c2, sizeof(char), 1, fin);
							fread(&c3, sizeof(char), 1, fin);
							fread(&c4, sizeof(char), 1, fin);
							int vDtmp = 0;
							vDtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;

							//printf("North, East, Down velocities (cm/s): %d, %d, %d\n", vNtmp, vEtmp, vDtmp);
							// print data to output file
							double vN = (double)vNtmp / 100;
							double vE = (double)vEtmp / 100;
							double vD = (double)vDtmp / 100;
							char buffer[256];
							sprintf(buffer, "%f, %f, %f\n", vN, vE, vD);
							//sprintf(buffer, "%d, %d, %d\n", vNtmp, vEtmp, vDtmp);
							fwrite(buffer, sizeof(char), strlen(buffer), fout1);
						}

					}
					else if (c1 == 0x01 && c2 == 0x07) { // ubx-nav-pvt frame found
						//printf("PVT message found\n");
						// skip all fields up to position
						int nbits_skip = 4 + 2 + 1 + 1 + 1 + 1 + 1 + 1 + 4 + 4 + 1 + 1 + 1 + 1;
						for (int u = 0; u < 24 + 2; u++) {
							fread(&c1, sizeof(char), 1, fin);
						}

						// read longitude (10^-7 degrees)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int lontmp = 0;
						lontmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double lon = (double)lontmp / pow(10, 7);

						// read latitude (10^-7 degrees)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int lattmp = 0;
						lattmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double lat = (double)lattmp / pow(10, 7);

						// read height above ellipsoid (mm)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int heighttmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double height = (double)heighttmp / 1000;

						// read height above msl (mm)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int hMSLtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
			   		    double hMSL = (double)hMSL / 1000;

						// skip accuracies
						for (int u = 0; u < 8; u++) {
							fread(&c1, sizeof(char), 1, fin);
						}

						// read velN (mm/s)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int velNtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double velN = (double)velNtmp / 1000;

						// read velE (mm/s)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int velEtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double velE = (double)velEtmp / 1000;

						// read velD (mm/s)
						fread(&c1, sizeof(char), 1, fin);
						fread(&c2, sizeof(char), 1, fin);
						fread(&c3, sizeof(char), 1, fin);
						fread(&c4, sizeof(char), 1, fin);
						int velDtmp = c4 << 24 | c3 << 16 | c2 << 8 | c1;
						double velD = (double)velDtmp / 1000;

						char buffer[1024];
						sprintf(buffer, "%f, %f, %f, %f, %f, %f\n", lat, lon, hMSL, velN, velE, velD);
						//sprintf(buffer, "%d, %d, %d, %d, %d, %d\n", lattmp, lontmp, hMSLtmp, velNtmp, velEtmp, velDtmp);
						fwrite(buffer, sizeof(char), strlen(buffer), fout2);
					}

				}
				c1 = c2;

			}

			fclose(fin);
			fclose(fout1);
			fclose(fout2);


			// free dynamically allocated memory
			memset(inputFile, 0, strlen(inputFile));
			memset(outputFile_vel, 0, strlen(outputFile_vel));
			memset(outputFile_PVT, 0, strlen(outputFile_PVT));
			memset(prefix, 0, strlen(prefix));

			free(inputFile);
			free(outputFile_vel);
			free(outputFile_PVT);
			free(prefix);
		}
		closedir(d);
	}

	return 0;


	
}
