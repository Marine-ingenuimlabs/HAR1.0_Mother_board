/*
+----------------------------------------------------------------------------
   Function   :  Compress_Numero_Lg 
+----------------------------------------------------------------------------
   Purpose    :  Compress Numero
   Parms      :  
   Returns    :  none                                                    
   Call Level :  Process Level
   Remarks    :
+----------------------------------------------------------------------------
*/

void Compress_Numero_Lg (uint8_t *source,uint8_t *dest, uint8_t lg)
{
   uin8_t  i, j, donnee;
   uint8_t k;
   k = 0;
   /* Compression de la memoire ou du bis */
   for (i=j=0; i<lg; i++)
   {
      /* Attention au cas particuliers du *, # et pause */
      switch(donnee = source[i])
      {
         case '*':
           donnee = 0x0A;
           break;
         case '#':
           donnee = 0x0B;
           break;
         /* case SHORT_R_KEY_VAL:    Rappel enregistreur a priori impossible */
         case '/':
           donnee = 0x0E;
           break;
         default:
           if(( donnee >= '0') && (donnee <= '9'))
              donnee -= '0';
           else
              donnee = 0xFF;
           break;
      }

      if ( donnee != 0xFF )
      {
         if(k % 2)
            dest[j++] |= donnee;
         else
            dest[j]    = donnee << 4;
         k++;
      }

      if( k == MAX_DIGIT_MEMORY )
         break;
   }

   if ( k == 0 ) /* on n'a aucun caractère de valide */
      return( 0 );

   /* On remplace la longueur par un marqueur de fin */
   if(k < MAX_DIGIT_MEMORY)
   {
     if(k % 2)
       dest[j] |= 0x0C;       /* bourrage */
     else
       dest[j]  = 0xCC;       /* bourrage */
     return( j + 1 );
   }
   else
     return(MAX_DIGIT_MEMORY/2);
}

/*
+----------------------------------------------------------------------------
   Function   :  decompress_Numero 
+----------------------------------------------------------------------------
   Purpose    :  Decompression des numeros charges en eeprom
   Parms      :  
   Returns    :  none                                                    
   Call Level :  Process Level
   Remarks    :
+----------------------------------------------------------------------------
*/
uint8_t tab_digit [] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                          '*', '#', 0, '/', '/', '/'};

void Decompress_Numero_Lg (uint8_t *dest, uint8_t *source, uint8_t lg)
{
   /* Decompression des numeros en eeprom (memoire ou bis) */
   uint8_t i, j, donnee;

   j = 0;

   for( i = 0 ; i < ( 2 * lg ) ; i++ )
   {
      donnee = source[j];
      if(i % 2)
      {
         donnee  &= 0x0F;
         j++;
      }
      else
         donnee >>= 4;
                                         /* bourrage donc fin de numero    */
      if( donnee == 0x0C)
         break;

      dest[i] = tab_digit[donnee];
   }

   return(i);
}


