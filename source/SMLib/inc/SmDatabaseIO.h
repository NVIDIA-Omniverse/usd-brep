// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmDatabaseIO.h
* PURPOSE: Header file for SmDatabaseIO and associated classes.
**********************************************************************/

#ifndef __SMDATABASEIO_H__
#define __SMDATABASEIO_H__

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#include <iostream>
#include <fstream>
#include <sstream>

#define SM_MAXSIZE  256  // max number of character per line in written files

/*******************************************************************//**
PURPOSE: A class to manage reading and writing values to streams in ASCII,
         Binary without byte swapping, and Binary with Byte swapping.

NOTES: The Begin, Write, Read, and End methods are all virtual
  so users can derive classes from this one to meet special needs.
***********************************************************************/
class SM_EXPORT SmDatabaseIO
{
 protected:
  SmFileType      m_eFileType ;      //  oneof: SM_ASCII,   // Database is an ASCII file
                                     //         SM_BINARY,  // Database is a Binary format of some type - the default is
                                     //                     // a binary file....or:
                                     //         SM_BYTESWAP // Binary with bytes swapped ( Big/Little Endian )

  std::ostream  * m_pStreamOut ;     // output file stream, target of all Write methods
  std::istream  * m_pStreamIn ;      // input file stream,  target of all Read methods
  bool            m_bByteSwap ;      // TRUE  = Swap value bytes before write or after read
                                     // FALSE = write values in current byte order
  bool            m_bOwnStream ;     // Only set true by new Style OpenFileForRead and OpenFileForWrite methods
                                     // TRUE  = (new style) close and delete streams when destructed
                                     // FALSE = (old style) don't
  ULONG           m_lVersion;        // File version number

 public:
  // default constructor
  SmDatabaseIO()                          : m_eFileType(SM_UNKNOWN),
                                            m_pStreamOut(NULL),
                                            m_pStreamIn(NULL),
                                            m_bByteSwap(IsBigEndian()),
                                            m_bOwnStream(FALSE),
                                            m_lVersion(30)
                                          { }

  // old style constructors
  SmDatabaseIO(SmFileType     eFileType,
               std::ostream * pStreamOut) : m_eFileType(eFileType),
                                            m_pStreamOut(pStreamOut),
                                            m_pStreamIn(NULL),
                                            m_bByteSwap((eFileType == SM_BYTESWAP) ? true : IsBigEndian()),
                                            m_bOwnStream(FALSE),
                                            m_lVersion(30)
                                          { }

  SmDatabaseIO(SmFileType     eFileType,
               std::istream * pStreamIn)  : m_eFileType(eFileType),
                                            m_pStreamOut(NULL),
                                            m_pStreamIn(pStreamIn),
                                            m_bByteSwap((eFileType == SM_BYTESWAP) ? true : IsBigEndian()),
                                            m_bOwnStream(FALSE),
                                            m_lVersion(30)
                                          { }

  SmDatabaseIO(SmFileType eFileType)      : m_eFileType(eFileType),
                                            m_pStreamOut(NULL),
                                            m_pStreamIn(NULL),
                                            m_bByteSwap((eFileType == SM_BYTESWAP) ? true : IsBigEndian()),
                                            m_bOwnStream(FALSE),
                                            m_lVersion(30)
                                          { }

  virtual ~SmDatabaseIO() {}

  // return TRUE if current system is running in BigEndian
  static bool IsBigEndian(void);

  // Swap value byte ordering
  double               ByteSwap128(double dBigDouble); // 64 bit
  double               ByteSwap64(double dDouble);     // 32 bit
  unsigned long        ByteSwap64(unsigned long nBiglong);
  unsigned long        ByteSwap32(unsigned long nLongNumber);
  unsigned long long   ByteSwap64(unsigned long long nBiglong);
  unsigned short       ByteSwap32(unsigned short nBigValue);
  unsigned short       ByteSwap16(unsigned short nValue);
  static std::uint32_t ByteSwap32(std::uint32_t  nLong );

  // simple data access
  SmFileType     GetFileType    ()                          { return m_eFileType; }
  std::ostream * GetOutStreamPtr()                          { return m_pStreamOut; }
  std::istream * GetInStreamPtr ()                          { return m_pStreamIn; }
  std::ostream * SetOutStreamPtr(std::ostream * pStreamOut) { m_pStreamOut = pStreamOut;  return m_pStreamOut; }
  std::istream * SetInStreamPtr (std::istream * pStreamIn)  { m_pStreamIn = pStreamIn;  return m_pStreamIn; }
  bool           GetByteSwap    ()                          { return m_bByteSwap; }
  void           SetByteSwap    (bool bByteSwap)            { m_bByteSwap = bByteSwap; }
  ULONG          GetVersionNum  ()                          { return m_lVersion; }
  void           SetVersionNum  ( ULONG lVersionNum )       { m_lVersion = lVersionNum; }

  // These abstract methods should be overwritten and never be called directly
  virtual SmStatus BeginWriting();
  virtual SmStatus EndWriting();
  virtual SmStatus BeginReading();
  virtual SmStatus EndReading();

  // default implementation for writing values
  virtual SmStatus WriteLongs     (const ULONG     * pLongs,    ULONG lNumLongs);
  virtual SmStatus WriteDoubles   (const double    * pDoubles,  ULONG lNumDoubles);
  virtual SmStatus WriteCharacters(const char      * pChars,    ULONG lNumChars);
  virtual SmStatus WriteBooleans  (const SmBoolean * pBooleans, ULONG lNumBooleans);
  virtual SmStatus WriteLong      (long      sLong);
  virtual SmStatus WriteBoolean   (SmBoolean bBoolean);
  virtual SmStatus WriteDouble    (double    dDouble);
  virtual SmStatus WriteShort     (short     sInt);
  virtual SmStatus WriteChar      (char      sChar);
  SmStatus WriteType(long lType, ULONG *pOptDim=NULL) ;

  // default implementation for reading values
  virtual SmStatus ReadLongs     (ULONG     * pLongs,    ULONG lNumLongs);
  virtual SmStatus ReadDoubles   (double    * pDoubles,  ULONG lNumDoubles);
  virtual SmStatus ReadCharacters(char      * pChars,    ULONG lNumChars);
  virtual SmStatus ReadBooleans  (SmBoolean * pBooleans, ULONG lNumBooleans);
  virtual SmStatus ReadLong      (long      & rLong);
  virtual SmStatus ReadLong      (ULONG     & rLong);
  virtual SmStatus ReadBoolean   (SmBoolean & rBoolean);
  virtual SmStatus ReadDouble    (double    & rDouble);
  virtual SmStatus ReadShort     (short     & rInt);
  virtual SmStatus ReadChar      (char      & rChar);
  SmStatus ReadType(long & lType, ULONG *pOptDim=NULL) ;
  void     GoToNextLine() { if(m_pStreamIn) { m_pStreamIn->ignore(SM_MAXSIZE, '\n') ; } }

  // for debug
  SmBoolean IsValidType(long & lType) ;

} ; // end class SmDatabaseIO

/*******************************************************************//**
PURPOSE: A class to manage Opening of files for Read/Write

NOTES:
***********************************************************************/
class SM_EXPORT SmDatabaseIOFile : public SmDatabaseIO
{
 public:
  SmDatabaseIOFile() { }

  // old style constructors
  SmDatabaseIOFile(SmFileType     eFileType,
                   std::ostream * pStreamOut) : SmDatabaseIO(eFileType, pStreamOut)
                                              { }

  SmDatabaseIOFile(SmFileType     eFileType,
                               std::istream * pStreamIn)  : SmDatabaseIO(eFileType, pStreamIn)
                                              { }

  SmDatabaseIOFile(SmFileType eFileType)      : SmDatabaseIO(eFileType)
                                              { }

  //  destructor
  ~SmDatabaseIOFile()                         { if (m_bOwnStream)
                                                 { if (m_pStreamOut != NULL)
                                                   { ((std::ofstream *)m_pStreamOut)->close() ;
                                                      delete m_pStreamOut ; m_pStreamOut = NULL ;
                                                    }
                                                   if (m_pStreamIn != NULL)
                                                    { ((std::ifstream *)m_pStreamIn)->close() ;
                                                       delete m_pStreamIn ;  m_pStreamIn = NULL ;
                                                     }
                                                  }
                                              }

  // new style open file for write - set all SmDatabaseIOFile values - destructor calls close()
  SmStatus OpenFileForWrite(const TCHAR  * cOutputFileName,    ///< [in] : target file name
                            SmFileType     eType = SM_ASCII,   ///< [in] : oneof: SM_ASCII    = write ascii file with comment lines
                                                               //             SM_BINARY   = write binary file from system format to LittleEndian
                                                               //             SM_BYTESWAP = write binary file - forcing byte swap
                            SmBoolean      bNewFile = FALSE) ; ///< [in] : TRUE  = open file and rewrite contents
                                                               //      FALSE = open file and append to end

  // new style open file for write - set all SmDatabaseIOFile values - destructor calls close()
  SmStatus OpenFileForRead(const TCHAR  * cInputFileName,     ///< [in] : target file name
                           SmFileType     eType = SM_ASCII);  ///< [in] : oneof: SM_ASCII    = read ascii file with comment lines
                                                              //             SM_BINARY   = read binary file from LittleEndian to system format
                                                              //             SM_BYTESWAP = read bineary file - forcing byte swap

} ; // end class SmDatabaseIOFile

/*******************************************************************//**
PURPOSE: A class to manage Opening of files for Read/Write

NOTES:
***********************************************************************/
class SM_EXPORT SmDatabaseIOStream : public SmDatabaseIO
{
 public:
  SmDatabaseIOStream() { }

  // old style constructors
  SmDatabaseIOStream(SmFileType           eFileType,
                     std::ostringstream * pStreamOut) : SmDatabaseIO(eFileType, pStreamOut)
                                                        { }
  SmDatabaseIOStream(SmFileType           eFileType,
                     std::istringstream * pStreamIn)  : SmDatabaseIO(eFileType, pStreamIn)
                                                        { }

  SmDatabaseIOStream(SmFileType eFileType)            : SmDatabaseIO(eFileType)
                                                        { }

  // simple data access
  std::ostream * GetOutStreamPtr()                                { return (std::ostringstream *)m_pStreamOut; }
  std::istream * GetInStreamPtr ()                                { return (std::istringstream *)m_pStreamIn; }
  std::ostream * SetOutStreamPtr(std::ostringstream * pStreamOut) { m_pStreamOut = pStreamOut;  return m_pStreamOut; }
  std::istream * SetInStreamPtr (std::istringstream  * pStreamIn) { m_pStreamIn  = pStreamIn;   return m_pStreamIn; }

  // new style open file for write - set all SmDatabaseIOStream values - destructor calls close()

} ; // end class SmDatabaseIOStream

#endif // !__SMDATABASEIO_H__
