#include <stdio.h>

#define TAILLE 500

float DistanceConsigne = 1.0;
float DistanceMax = 0.5;
float DistanceMin = 0.0;
float DeltaDc = 0.1;
float Distance[TAILLE];

float VitesseConsigne = 0.0;
float VitesseConsignePrecedente = 0.0;
float VitesseMax = 1.2;

float AccelerationMax = 3.5;
float DeccelerationMax = 3.5;

float KpPosition = 5.3;
float KpPositionDepart = 5.3;
float DeltaKp = 0.1;

float Te = 0.01;

int main()
{	
	int i, j, k;
	
	int CoefTrouve = 0;
	
	printf("Programme de calibration du coefficient proportionnel donnant la consigne de vitesse a partir de l'erreur de distance\n");
	
	printf("Entrez les reglages du robot en SI : (s, m, m/s, m/s²) ou (s, rad, rad/s, rad/s²) :\n");
	printf("Temps echantillonage =");scanf("%g", &Te);
	printf("Vitesse Max =");scanf("%g", &VitesseMax);
	printf("Acceleration Max =");scanf("%g", &AccelerationMax);
	printf("Decceleration Max =");scanf("%g", &DeccelerationMax);
	
	printf("Entrez les parametres de la calibration en SI : (s, m, m/s, m/s²) ou (s, rad, rad/s, rad/s²) :\n");
	printf("Distance Mini =");scanf("%g", &DistanceMin);
	printf("Echantillonage distance =");scanf("%g", &DeltaDc);
	printf("Distance Max =");scanf("%g", &DistanceMax);
	printf("Coefficient minimum =");scanf("%g", &KpPositionDepart);
	printf("Echantillonage coefficient =");scanf("%g", &DeltaKp);
	
	FILE * F = fopen("resultat.txt", "w");
	
	DistanceConsigne = DistanceMin;
	
	while(DistanceConsigne < DistanceMax)
	{
		DistanceConsigne += DeltaDc;
	
		CoefTrouve = 0;
	
		KpPosition = KpPositionDepart;
		
		while(CoefTrouve == 0)
		{
			KpPosition += DeltaKp;
			
			Distance[0] = 0;
			
			for(i = 1; i < TAILLE; i++)
			{			
				VitesseConsignePrecedente = VitesseConsigne;
				VitesseConsigne = KpPosition * (DistanceConsigne - Distance[i-1]);
				
				if(VitesseConsigne > VitesseMax)
					VitesseConsigne = VitesseMax;
					
				if(VitesseConsigne < -VitesseMax)
					VitesseConsigne = -VitesseMax;
				
				if((VitesseConsigne - VitesseConsignePrecedente) / Te > AccelerationMax)
					VitesseConsigne = VitesseConsignePrecedente + AccelerationMax * Te;
					
				if((VitesseConsigne - VitesseConsignePrecedente) / Te < -DeccelerationMax)
					VitesseConsigne = VitesseConsignePrecedente - DeccelerationMax * Te;
					
				Distance[i] = (VitesseConsigne + VitesseConsignePrecedente) * Te / 2.0 + Distance[i-1];
			}	
			
			for(j = 0; j < TAILLE; j++)
				if(Distance[j] > DistanceConsigne)
					CoefTrouve = 1;
				
			if(CoefTrouve == 1)
				KpPosition -= DeltaKp;
		}
		
		fprintf(F, "%f\t%f\n", DistanceConsigne, KpPosition);
	}
		
	fclose(F);

	printf("Les resultats sont stockes dans le fichier resultat sous la forme : DistanceConsigne [tab] CoefficientOptimise\n");
	
	system("pause");
}	
