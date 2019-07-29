

using Com.Nelalen.GameObject;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using TestBepuPhysic;

namespace Com.Nelalen.GameObject
{
    internal class PlayerCharacter : Character
    {
        private int charId;
        
        private int characterId { get { return charId; } set { charId = value; } }
        private int clientId;

        internal PlayerCharacter(int characterId, int clientId, string name, int unitId, System.Numerics.Vector3 startPosition, Map map) : base(unitId, name, startPosition, map )
        {
            collider.type = Collider.Type.PlayerCharacer;
            this.charId = characterId;
            this.clientId = clientId;
        }


        internal int GetClientId() {
            return clientId;
        }

        internal int GetCharId() {
            return charId;
        }

    }
}
