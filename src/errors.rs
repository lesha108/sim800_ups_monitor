use core::str::Utf8Error;

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    // invalid UTF-8.
    EncodingError,
    // что то моё
    UPSFailure,
    SerialError,
    TimerError,
    SerialNoData,
    FmtError,
    CmdFail,
    SmsStage1,
    SmsStage2,
    NoRing,
    NotAuthCall,
    NoAuthNumbers,
    NoInSMS,
    EspOffLine,
    EepromFail,
    NoUCS2,
    InvalidUCS2Size,

}

impl From<Utf8Error> for Error {
    fn from(_: Utf8Error) -> Self {
        Error::EncodingError
    }
}

impl From<core::fmt::Error> for Error {
    fn from(_: core::fmt::Error) -> Self {
        Error::FmtError
    }
}
