// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************//**
* FILE NAME --- SmDatabaseIO.cpp
* PURPOSE:
**********************************************************************/
/*___*/


#include "StdAfx.h"

#include <SmDatabaseIO.h>
#include <SmCurveTypes.h>
#include <SmSurfTypes.h>
#include <SmVolumeTypes.h>
#include <limits.h>

// Note:  BigEndian machines  ( IBM/SPARC/MAC) will store binary
// SMS data in Little-endian format , and all reads/writes will
// do a ByteSwap (except char).  Microsoft etc is Little-endian.
// see http://www.codeproject.com/cpp/endianness.asp?print=true

// Note: Under UNICODE, char may be 2 bytes not 1 ( we test sizeof() )

bool SmDatabaseIO::IsBigEndian(void)
// Will return true if this is a big-endian machine, else false
{
 short word = 0x4321;
   if((*(char *)& word) != 0x21 )
     return true;
   else
     return false;
}

double  SmDatabaseIO::ByteSwap128(double dDouble)
  // use for 128 bit doubles
  // swap by char the slow way
{ 
    ULONG lSize = sizeof(double);
    unsigned char *cptr, tmp;
    cptr =(unsigned char *)&dDouble;
    for(ULONG ii=0;ii<(lSize/2);ii++)
    {
      tmp = cptr[ii];
      cptr[ii] = cptr[(lSize-1)-ii];
      cptr[(lSize-1)-ii] = tmp;
    }
    return (dDouble);
}

double  SmDatabaseIO::ByteSwap64(double dDouble)
// use for 64 bit doubles
// swap by char the slow way
{ 
    ULONG lSize = sizeof(double);
    unsigned char *cptr, tmp;
    cptr =(unsigned char *)&dDouble;
    for (ULONG ii=0;ii<(lSize/2);ii++)
    {
      tmp = cptr[ii];
      cptr[ii] = cptr[(lSize-1)-ii];
      cptr[(lSize - 1)-ii] = tmp;
    }

    return (dDouble);
}

unsigned long long  SmDatabaseIO::ByteSwap64(unsigned long long nBiglong)
  // use for 64 bit long longs
  // swap by char the slow way
{
    ULONG lSize = sizeof(unsigned long long);
    unsigned char *cptr, tmp;
    cptr =(unsigned char *)& nBiglong;
    for (ULONG ii=0; ii<(lSize/2); ii++)
    {
      tmp = cptr[ii];
      cptr[ii] = cptr[(lSize-1)-ii];
      cptr[(lSize - 1)-ii] = tmp;
    }

    return (nBiglong);
}

unsigned long  SmDatabaseIO::ByteSwap64(unsigned long nBiglong)
// use for 64 bit longs
// swap by char the slow way
{ 
    ULONG lSize = sizeof(unsigned long);
    unsigned char *cptr, tmp;
    cptr =(unsigned char *)& nBiglong;
    for (ULONG ii=0; ii<(lSize/2); ii++)
    {
      tmp = cptr[ii];
      cptr[ii] = cptr[(lSize-1)-ii];
      cptr[(lSize-1)-ii] = tmp;
    }

    return (nBiglong);
}

unsigned long SmDatabaseIO::ByteSwap32(unsigned long nLongNumber)
// use for 32 bit integers only (4byte)
// we may have problems with sign bits for long and short
// see http:// www.dmh2000.com/cpp/dswap.shtml
{
  return (((nLongNumber&0x000000FF) << 24) + ((nLongNumber&0x0000FF00) << 8)+
          ((nLongNumber&0x00FF0000) >> 8) +((nLongNumber&0xFF000000) >> 24));
}

unsigned short SmDatabaseIO::ByteSwap32(unsigned short nLongNumber)
// use for 32 bit short
{
  return (((nLongNumber&0x000000FF) << 24) + ((nLongNumber&0x0000FF00) << 8)+
        ((nLongNumber&0x00FF0000) >> 8) +((nLongNumber&0xFF000000) >> 24));
}

uint32_t SmDatabaseIO::ByteSwap32(uint32_t nLong)
// use for 32 bit integers only (4byte)
// we may have problems with sign bits for long and short
// see http:// www.dmh2000.com/cpp/dswap.shtml
{
  return (((nLong&0x000000FF) << 24) + ((nLong&0x0000FF00) << 8)+
          ((nLong&0x00FF0000) >> 8) +((nLong&0xFF000000) >> 24));
}

unsigned short SmDatabaseIO::ByteSwap16(unsigned short nValue)
{
    return (((nValue>> 8)) |(nValue << 8));
}

/**********************************************************************//*
PURPOSE: Open file for write saving stream pointer and data internally.

NOTES: When writing a SM_BINARY file, the current system byte ordering
  (BigEndian or LittleEndian) is checked and the m_bByteSwap bit is
  set as needed for all output to be in LittleEndian format.

  That matches the behavior of the OpenFileForRead() method.
***********************************************************************/
SmStatus SmDatabaseIOFile::OpenFileForWrite
 (const TCHAR  * cOutputFileName,    // in : target file name
  SmFileType     eType,              // in : oneof: SM_ASCII    = write ascii file with comment lines
                                     //             SM_BINARY   = write binary file from system format to LittleEndian
                                     //             SM_BYTESWAP = write binary file - forcing byte swap
                                     //      default:[SM_ASCII]
  SmBoolean      bNewFile)           // in : TRUE  = open file and rewrite contents
                                     //      FALSE = open file and append to end
                                     //      default:[FALSE]
{
  // check state
    if (m_pStreamOut != NULL) { ((std::ofstream *)m_pStreamOut)->close(); delete m_pStreamOut; m_pStreamOut = NULL; }
    if (m_pStreamIn != NULL) { ((std::ifstream *)m_pStreamIn)->close(); delete m_pStreamIn; m_pStreamIn = NULL; }

  // get a stream
  m_pStreamOut = new std::ofstream() ;

  // save input params
  m_eFileType = eType;
  m_bByteSwap = (eType == SM_BYTESWAP) ? true : IsBigEndian();
  m_bOwnStream = TRUE;

  // open file for output
  if (eType == SM_ASCII)
  {
      if (bNewFile) { ((std::ofstream *)m_pStreamOut)->open(cOutputFileName, SM_IOS::out | SM_IOS::trunc); }
      else          { ((std::ofstream *)m_pStreamOut)->open(cOutputFileName, SM_IOS::out | SM_IOS::app); }
      m_pStreamOut->setf(SM_IOS::fixed);
      m_pStreamOut->precision(16);
    }
  else    // SM_BINARY
  {
      if (bNewFile) { ((std::ofstream *)m_pStreamOut)->open(cOutputFileName, SM_IOS::out | SM_IOS::trunc | SM_IOS_BINARY); }
      else          { ((std::ofstream *)m_pStreamOut)->open(cOutputFileName, SM_IOS::out | SM_IOS::app | SM_IOS_BINARY); }
    }

  // check file open state
  if (m_pStreamOut == NULL || !m_pStreamOut->good())
    {
      if( m_pStreamOut ) { delete m_pStreamOut ; m_pStreamOut = NULL ; }

      TCHAR sBuff[SM_TBLOCK_SIZE] ;
      smos_sprintf(sBuff,_T("SmDatabaseIOFile::OpenForWrite(\"%s\") : stream::open() failed"), cOutputFileName) ;

      SER_MSG(SM_ERR_INVALID_INPUT, sBuff);
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmDatabaseIOFile::OpenFileForWrite

/**********************************************************************//*
PURPOSE: Open file for read saving stream pointer and data internally.

NOTES: When reading a SM_BINARY file, the current system byte ordering
  (BigEndian or LittleEndian) is checked and the m_bByteSwap bit is
  set as needed for all input to be read in LittleEndian format but
  saved in the current system's byte ordering format.

  That matches the behavior of the OpenFileForWrite() method.
***********************************************************************/
SmStatus SmDatabaseIOFile::OpenFileForRead
 (const TCHAR  * cInputFileName,     // in : target file name
  SmFileType     eType)              // in : oneof: SM_ASCII    = read ascii file with comment lines
                                     //             SM_BINARY   = read binary file from littleEndian to system format
                                     //             SM_BYTESWAP = read binary file - forcing byte swap
                                     //      default:[SM_ASCII]
{
  // check state
    if (m_pStreamOut != NULL) { ((std::ofstream *)m_pStreamOut)->close(); delete m_pStreamOut; m_pStreamOut = NULL; }
    if (m_pStreamIn != NULL) { ((std::ifstream *)m_pStreamIn)->close(); delete m_pStreamIn; m_pStreamIn = NULL; }

  // get a stream
  m_pStreamIn = new std::ifstream() ;

  // save input params
  m_eFileType = eType ;
  m_bByteSwap = (eType == SM_BYTESWAP) ? true : IsBigEndian() ;
  m_bOwnStream = TRUE ;

  // open stream for ASCII or BINARY input
  if (eType == SM_ASCII) { ((std::ifstream *)m_pStreamIn)->open(cInputFileName, SM_IOS::in | SM_IOS_NOCREATE); }
  else /* SM_BINARY */   { ((std::ifstream *)m_pStreamIn)->open(cInputFileName, SM_IOS::in | SM_IOS_BINARY | SM_IOS_NOCREATE); }

  // check the stream
  if(m_pStreamIn->fail() != 0 || !m_pStreamIn->good())
    {
      if( m_pStreamIn ) { delete m_pStreamIn ; m_pStreamIn = NULL ; }

      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("Failed SmBrepData::ReadFromFile(\"%s\") : std::istream::open()"), cInputFileName) ;

      SER_MSG(SM_ERR_INVALID_INPUT, sBuff);
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmDatabaseIOFile::OpenFileForRead

// These abstract methods should be overwritten and never be called directly
SmStatus SmDatabaseIO::BeginWriting() { return SM_SUCCESS; }
SmStatus SmDatabaseIO::EndWriting()   { return SM_SUCCESS; }
SmStatus SmDatabaseIO::BeginReading() { return SM_SUCCESS; }
SmStatus SmDatabaseIO::EndReading()   { return SM_SUCCESS; }

// default implementation for reading and writing values
SmStatus SmDatabaseIO::WriteLongs(const ULONG * pLongs, ULONG lNumLongs)
{
	NER(m_pStreamOut);

	for (ULONG n = 0; n < lNumLongs; n++)
	{
		unsigned long long llLong = (unsigned long long)pLongs[n];
		if (m_bByteSwap)
		{
			llLong = ByteSwap64((unsigned long long) &llLong);
		}
		m_pStreamOut->write((char *)&llLong, sizeof(unsigned long long));
	}

	return (SM_SUCCESS);
}

SmStatus SmDatabaseIO::WriteDoubles(const double * pDoubles, ULONG lNumDoubles)
{
    NER(m_pStreamOut);
    if (m_bByteSwap)
    {
      for (ULONG n = 0; n < lNumDoubles; n++)
        {
          WriteDouble(pDoubles[n]); // passes a copy of double (not reference)
        }
      return (SM_SUCCESS);
    }

    m_pStreamOut->write((char *)pDoubles, sizeof(double)*lNumDoubles);
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteCharacters(const char * pChars, ULONG lNumChars)
{
    NER(m_pStreamOut);
    if (m_bByteSwap && (sizeof(char) == 2))
        // note under UNICODE chars may be 2 bytes
    {
      for (ULONG n = 0; n < lNumChars; n++)
        {
          WriteChar(pChars[n]); // passes a copy of each char
        }
      return (SM_SUCCESS);
    }

    m_pStreamOut->write((char*)pChars, sizeof(char)*lNumChars);
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteBooleans(const SmBoolean * pBooleans, ULONG lNumBooleans)
{
    NER(m_pStreamOut);
    if (m_bByteSwap)
    {
      for (ULONG n = 0; n < lNumBooleans; n++)
        {
          WriteBoolean(pBooleans[n]); // passes a copy of SmBoolean (not reference)
        }
      return (SM_SUCCESS);
    }

    m_pStreamOut->write((char *)pBooleans, sizeof(SmBoolean)*lNumBooleans);
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteLong(long lLong)
{
    NER(m_pStreamOut);

    long long llLong = (long long) lLong;

    if (m_bByteSwap)
    { llLong = ByteSwap64((unsigned long long) &llLong); }

    m_pStreamOut->write((char *) &llLong, sizeof( long long) );

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteDouble(double dDouble)
// 'Most computers use the IEEE format for doubles, regardless of
// whether they are big or little endian for integers.'
{
    NER(m_pStreamOut);
    if (m_bByteSwap)
    {
      dDouble = (sizeof(double) == 8) ?
            ByteSwap64(dDouble):
            ByteSwap128(dDouble);
    }

    m_pStreamOut->write((char *)&dDouble, sizeof(double));
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteBoolean(SmBoolean bBoolean)
{
    NER(m_pStreamOut);
    m_pStreamOut->write((char *)&bBoolean, sizeof(SmBoolean));
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteShort(short sInt)
{
    NER(m_pStreamOut);
    if (m_bByteSwap)
    {
      sInt = (sizeof(short) == 2) ?
            ByteSwap16((unsigned short &)sInt) :
            ByteSwap32((unsigned short &)sInt);
    }

    m_pStreamOut->write((char *)&sInt, sizeof(short));
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteChar(char sChar)
{
    NER(m_pStreamOut);
    if (m_bByteSwap && (sizeof(char) == 2))
    {
      sChar = (char) ByteSwap16((char &)sChar);
    }

    m_pStreamOut->write((char *)&sChar, sizeof(char));
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::WriteType(long lType,     // in : type value to output
                                 ULONG *pOptDim) // in : used for writing curve types, NULL to ignore, default:[NULL]
{
#ifdef SM_DEBUG_CODE
  if(!IsValidType(lType))
    {
      SM_ASSERT_MSG(IsValidType(lType), _T("SmDatabaseIOFile::WriteType() Wrote an unsupported user type")) ;
    }
#endif // SM_DEBUG_CODE

  if(m_eFileType == SM_ASCII) { *m_pStreamOut << "Object Type " << lType << "\n";   // "Object"  for SM_CURRENT_DATABASE_VERSION >= 34
                                                                                    // "Curve or Surface" for SM_CURRENT_DATABASE_VERSION < 34
                                if(pOptDim) { *m_pStreamOut << "Curve Dim " << *pOptDim << "\n"; }
                              }
  else                        { SER(WriteLong(lType)) ;
                                if(pOptDim) { WriteLong(*pOptDim) ; }
                              }
  return(SM_SUCCESS) ;
}

SmStatus SmDatabaseIO::ReadType(long & rlType,   // in : type object to load from read
                                ULONG *pOptDim)  // in : used for reading curve types, NULL to ignore, default:[NULL]
{
  if(m_eFileType == SM_ASCII) { char sBuff[SM_MAXSIZE];

                                // Object Type:
                                *m_pStreamIn >> sBuff >> sBuff >> rlType ;  GoToNextLine() ;

                                // optional dimension
                                if(pOptDim)
                                  { *m_pStreamIn >> sBuff >> sBuff >> *pOptDim ; GoToNextLine() ; }
                              }
  else                        { SER(ReadLong(rlType)) ;

                                if(pOptDim)
                                  { SER(ReadLong(*pOptDim)) ; }
                              }


#ifdef SM_DEBUG_CODE
  if(!IsValidType(rlType))
    {
      SM_ASSERT_MSG(IsValidType(rlType), _T("SmDatabaseIOFile::ReadType() Read an unsupported user type")) ;
    }
#endif // SM_DEBUG_CODE

  return(SM_SUCCESS) ;
}

SmBoolean SmDatabaseIO::IsValidType(long &lType)
{
  return(   lType == SmCurve_TYPE
         || lType == SmLine_TYPE
         || lType == SmConic_TYPE
         || lType == SmCircle_TYPE
         || lType == SmEllipse_TYPE
         || lType == SmParabola_TYPE
         || lType == SmHyperbola_TYPE
         || lType == SmCompositeCurve_TYPE
         || lType == SmBSplineCurve_TYPE
         || lType == SmHermiteCurve_TYPE
         || lType == SmOffsetCurve_TYPE
         || lType == SmProjectedCurve_TYPE
         || lType == SmCompositeCurveRegion_TYPE
         || lType == SmCompositeCurveSegment_TYPE
         || lType == SmIsoCurve_TYPE
         || lType == SmCrvOnSurf_TYPE
         || lType == SmTangentField_TYPE
         || lType == SmCrvInVolume_TYPE

         || lType == SmSurface_TYPE
         || lType == SmBSplineSurface_TYPE
         || lType == SmPlane_TYPE
         || lType == SmCone_TYPE
         || lType == SmCylinder_TYPE
         || lType == SmSphere_TYPE
         || lType == SmTorus_TYPE
         || lType == SmSurfOfRevolution_TYPE
         || lType == SmSurfOfExtrusion_TYPE
         || lType == SmBlendSurface_TYPE
         || lType == SmCurveBoundedSurface_TYPE
         || lType == SmSrfInVolume_TYPE
         || lType == SmOffsetSurface_TYPE
         || lType == SmSTEPSurface_TYPE
         || lType == SmVolume_TYPE
         || lType == SmBSplineVolume_TYPE
         || lType == SmBendVolume_TYPE
         || lType == SmTransform_TYPE
         || lType == SmUnbendVolume_TYPE
         || lType == SmTwistVolume_TYPE) ;

} // end SmDatabaseIOFile::IsValidType

SmStatus SmDatabaseIO::ReadLongs(ULONG * pLongs, ULONG lNumLongs)
{
    NER(m_pStreamIn);
    if ( GetVersionNum() >= 40 )
    {
        unsigned long long * pllLongs = new unsigned long long [lNumLongs];
        unsigned long long   llTemp;

        m_pStreamIn->read( (char *) pllLongs, sizeof( unsigned long long )*lNumLongs );

        for ( ULONG n = 0; n < lNumLongs; n++ )
        {
            if ( m_bByteSwap )
            { llTemp = ByteSwap64( pllLongs[n] ); }
            else
            { llTemp = pllLongs[n]; }

            if ( llTemp > ULONG_MAX )
            { SER( SM_ERR ); }
            else
            { pLongs[n] = (ULONG)llTemp; }
        }
        delete[] pllLongs;
    }
    else
    {
        m_pStreamIn->read((char*)pLongs, sizeof(ULONG)*lNumLongs);
        if (m_bByteSwap)
        {
            for (ULONG n = 0; n < lNumLongs; n++)
            {
        unsigned long temp = (sizeof(long) == 4) ?
              ByteSwap32((unsigned long)(pLongs[n])) :
              ByteSwap64((unsigned long)(pLongs[n]));
                pLongs[n] = temp;
            }
        }
    }

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadDoubles(double * pDoubles, ULONG lNumDoubles)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)pDoubles, sizeof(double)*lNumDoubles);
    if (m_bByteSwap)
    {
      for (ULONG n = 0; n < lNumDoubles; n++)
        {
          double temp = pDoubles[n];
          pDoubles[n] = (sizeof(double) == 8) ?
                ByteSwap64(temp):
                ByteSwap128(temp);
        }
    }

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadCharacters(char * pChars, ULONG lNumChars)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)pChars, sizeof(char)*lNumChars);
    if (m_bByteSwap && sizeof(char) == 2)
    {
      for (ULONG n = 0; n < lNumChars; n++)
        {
          char temp = pChars[n];
          pChars[n] = (char) ByteSwap16((unsigned short)temp);
        }
    }    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadBooleans(SmBoolean * pBooleans, ULONG lNumBooleans)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)pBooleans, sizeof(SmBoolean)*lNumBooleans);
    if (m_bByteSwap)
    {
      for (ULONG n = 0; n < lNumBooleans; n++)
        {
          SmBoolean temp = pBooleans[n];
          pBooleans[n] =   (sizeof(SmBoolean) == 2) ? ByteSwap16((unsigned short)temp)
                         : (sizeof(SmBoolean) == 4) ? ByteSwap32((unsigned long)temp)
                         : (sizeof(SmBoolean) == 8) ? ByteSwap64((unsigned long)temp)
                         : (SmBoolean)ByteSwap128((double)temp);
        }
    }

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadLong(long & rLong)
{
    NER(m_pStreamIn);
    if ( GetVersionNum() >= 40 )
    {
        long long llLong, llTemp;
        m_pStreamIn->read( (char *) & llLong, sizeof(long long) );
        if (m_bByteSwap)
        { llTemp = ByteSwap64( (unsigned long long) llLong ); }
        else
        { llTemp = llLong; }

        if ( llTemp > LONG_MAX )
        { SER( SM_ERR ); }
        else
        { rLong = (ULONG)llTemp; }
    }
    else
    {
        m_pStreamIn->read((char *)& rLong, sizeof(long));
    if (m_bByteSwap)
        {
            unsigned long temp;
            temp = (sizeof(long) == 4) ?
                  ByteSwap32((unsigned long)rLong):
                  ByteSwap64((unsigned long)rLong);
            rLong = temp;
        }
    }
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadLong(ULONG & rLong)
{
    NER(m_pStreamIn);
    if ( GetVersionNum() >= 40 )
    {
        unsigned long long llLong, temp;
        m_pStreamIn->read((char *)&llLong, sizeof(unsigned long long));

        if (m_bByteSwap)
        { temp = ByteSwap64(llLong); }
        else 
        { temp = llLong; }

        if ( temp > ULONG_MAX )
        { SER( SM_ERR ); }
        else
        { rLong = (ULONG)temp; }
    }
    else
    {
        m_pStreamIn->read((char *)&rLong, sizeof(ULONG));
    if (m_bByteSwap)
        {
            unsigned long temp;
            temp = (sizeof(long) == 4) ?
                  ByteSwap32((unsigned long)rLong):
                  ByteSwap64((unsigned long)rLong);
            rLong = temp;
        }
    }

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadBoolean(SmBoolean & bBoolean)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)&bBoolean, sizeof(SmBoolean));
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadDouble(double & rDouble)
// Most computers use the IEEE format for doubles, regardless
// of whether they are big or little endian for integers.
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)&rDouble, sizeof(double));
    if (m_bByteSwap)
    {
      double temp;
      temp = (sizeof(double) == 8) ?
            ByteSwap64(rDouble):
            ByteSwap128(rDouble);
      rDouble = temp;
    }

    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadShort(short & rInt)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)&rInt, sizeof(short));
    if (m_bByteSwap)
    {
      short temp;
      temp = (sizeof(short) == 2) ?
            ByteSwap16((unsigned short)rInt):
            ByteSwap32((unsigned short)rInt);
      rInt = temp;
    }
    return SM_SUCCESS;
}

SmStatus SmDatabaseIO::ReadChar(char & rChar)
{
    NER(m_pStreamIn);
    m_pStreamIn->read((char *)&rChar, sizeof(char));
    if (m_bByteSwap  && (sizeof(char) == 2))
    {
      char temp = (char) ByteSwap16((unsigned short)rChar);
      rChar = temp;
    }
    return SM_SUCCESS;
}
