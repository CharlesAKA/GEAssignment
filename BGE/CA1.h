#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class CA1:
		public Game
	{
	private:
		
	public:
		CA1(void);
		~CA1(void);
		bool Initialise();
		void Update();
		void Cleanup();
		void CreateWall();
	};
}
