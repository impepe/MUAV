#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define SHM_ADDR 0x100
#define WRITE 0
#define READ  1

struct shared_str
{	
	char text0[4];
	char text1[4];
	char text2[4];
	char text3[17];
};

int main()
{
	//printf("%d\n",sizeof(struct shared_str));

	int shm_id;
	shm_id = shmget((key_t)SHM_ADDR, sizeof(struct shared_str), 0666|IPC_CREAT);
	if (shm_id == -1)	
	{
		printf("failed to creat shared memory\n");
		return 0;
	}

	void *shm = NULL;
	shm = shmat(shm_id, 0, 0);
	if (shm == (void *)-1)	
	{
		printf ("failed to connect shared memory\n");
		return 0;
	}

	struct shared_str *shared = (struct shared_str *)shm;
	
	printf("start py\n");
	system("python3 shm_py.py");
	sleep(1);
	
	float f1 = atof(shared->text0);
	float f2 = atof(shared->text1);
	float f3 = atof(shared->text2);
	printf("%f\n%f\n%f\n",f1,f2,f3);

	return 0;
/*
	int i=0;
	while (1)
	{
		if (shared->rw == READ)
		{
			printf("ref_x = %f\nref_y = %f\nref_z = %f\n",shared->ref_x,shared->ref_y,shared->ref_z);
			shared->rw = WRITE;
			sleep(2000);
			i++;
			if (i>3)	break;
		}

		else	sleep(1);
	}

	if (shmdt(shm) == -1)
	{
		printf("shmdt failed\n");
		exit(EXIT_FAILURE);
	}

	if (shmctl(shm_id, IPC_RMID, 0) == -1)
	{
		printf("delete shm failed\n");
		exit(EXIT_FAILURE);
	}
*/
	printf("success\n");
	exit(EXIT_SUCCESS);

}























