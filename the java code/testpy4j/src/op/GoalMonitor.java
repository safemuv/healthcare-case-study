package op;

public class GoalMonitor implements Runnable {

	@Override
	public void run() {
		while(true) {
			System.out.println();
		}
		
	}
	
	public void StartMonitor() {
		Thread h1=new Thread(this);
		
		try {
			h1.start();
			h1.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	

}

/*buscra la menra de conectar de recibir el eststus del goal en el hilo e imprimir*/