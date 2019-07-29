using System;
using System.Collections.Generic;
using System.Text;

namespace Com.Nelalen.GameObject
{
    internal struct Collider
    {
        internal enum Type { Collidable, Unit, Characer, PlayerCharacer }
        internal Type type;

        internal bool isPassThrough;
        internal int bodyHandle;

    }
}
