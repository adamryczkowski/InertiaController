# InertiaController
Library for dynamic updating of state of 1-dimensional slider with simulated inertia.

Usage:

``` C++
const float maxV = 50;

InertiaController contr( 10, maxV);
InertiaPhysics phys(NULL, 30, 0, 120,50,10);

void UpdatePhysics();
void UpdateControler();

void UpdatePhysics()
{
	phys.update(millis());
}

void UpdateControler()
{
//	contr.setTarget(encoderPos);  //to set a new target
	uint32_t time = contr.updatePhysics(phys);
}

```

