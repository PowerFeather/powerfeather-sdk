

namespace PowerFeather
{
    enum class Result
    {
        Ok = 0,
        Failure = -1,
        InvalidState = -2,
        Timeout = -3,
        InvalidArg = -4,
        NotReady = -5
    };
}