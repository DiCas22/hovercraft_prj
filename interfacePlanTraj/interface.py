import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import os
import re
import json
import paho.mqtt.client as mqtt

PASTA_ROTAS = "rotas"
rotas_para_envio = []

if not os.path.exists(PASTA_ROTAS):
    os.makedirs(PASTA_ROTAS)


def sanitize_filename(name: str) -> str:
    # remove caracteres que não são letras, números, espaço, traço ou underscore
    name = name.strip()
    name = re.sub(r"[^\w\-\s]", "", name)
    name = name.replace(" ", "_")
    return name if name else None


class CriadorRotaSetas:
    def __init__(self, root):
        self.root = root
        self.root.title("Sistema de Rotas do Caminhão Autônomo")
        self.GRID_MIN = 0
        self.GRID_MAX = 9
        self.PONTOS_ESPECIAIS = {
            "Início": (0, 0),
            "Destino 1": (8, 1),
            "Destino 2": (9, 9)
        }
        
        # MQTT Comunicação
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        try:
            self.mqtt_client.connect("broker.hivemq.com", 1883, 60)
            self.mqtt_client.loop_start()
        except:
            print("Falha ao conectar no broker MQTT.")

        # ----- MENU LATERAL -----
        menu_frame = tk.Frame(root)
        menu_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        tk.Button(menu_frame, text="Criar Nova Rota", width=20,
                  command=self.criar_nova_rota).pack(pady=5)

        tk.Button(menu_frame, text="Salvar Rota", width=20,
                  command=self.salvar_rota).pack(pady=5)

        tk.Button(menu_frame, text="Carregar Rota", width=20,
                  command=self.carregar_rota).pack(pady=5)

        tk.Button(menu_frame, text="Enviar Rota Selecionada", width=20,
                  command=self.enviar_rota).pack(pady=5)

        tk.Label(menu_frame, text="Rotas Salvas:").pack(pady=10)

        # Lista de rotas
        self.lista_rotas = tk.Listbox(menu_frame, width=25, height=20)
        self.lista_rotas.pack()
        self.lista_rotas.bind("<Double-Button-1>", lambda e: self.carregar_rota())
        self.atualizar_lista()

        # ----- ÁREA DO GRÁFICO -----
        graficos_frame = tk.Frame(root)
        graficos_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig, self.ax = plt.subplots(figsize=(6, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=graficos_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.pontos = []
        self.x = 0
        self.y = 0

        root.bind("<Up>", self.mover)
        root.bind("<Down>", self.mover)
        root.bind("<Left>", self.mover)
        root.bind("<Right>", self.mover)

        self.modo_criacao = False

    # ---------------------------------------------------------
    def criar_nova_rota(self):
        self.modo_criacao = True
        self.x, self.y = self.PONTOS_ESPECIAIS["Início"]
        self.pontos = [self.PONTOS_ESPECIAIS["Início"]]
        self.atualizar_grafico()
        messagebox.showinfo("Modo Criação", "Início no ponto (0,0).\nUse as setas para construir a rota.")

    # ---------------------------------------------------------
    def mover(self, event):
        if not self.modo_criacao:
            return

        novo_x, novo_y = self.x, self.y

        if event.keysym == "Up":
            novo_y += 1
        elif event.keysym == "Down":
            novo_y -= 1
        elif event.keysym == "Left":
            novo_x -= 1
        elif event.keysym == "Right":
            novo_x += 1

        if not (self.GRID_MIN <= novo_x <= self.GRID_MAX and
                self.GRID_MIN <= novo_y <= self.GRID_MAX):
            return

        self.x, self.y = novo_x, novo_y
        self.pontos.append((self.x, self.y))
        self.atualizar_grafico()

    # ---------------------------------------------------------
    def atualizar_grafico(self):
        self.ax.clear()

        # --- GRID FIXO ---
        self.ax.set_xticks(range(self.GRID_MIN, self.GRID_MAX + 1))
        self.ax.set_yticks(range(self.GRID_MIN, self.GRID_MAX + 1))
        self.ax.grid(True)

        self.ax.set_xlim(self.GRID_MIN - 0.5, self.GRID_MAX + 0.5)
        self.ax.set_ylim(self.GRID_MIN - 0.5, self.GRID_MAX + 0.5)

        # --- DESENHAR ROTA ---
        if self.pontos:
            xs, ys = zip(*self.pontos)
            self.ax.plot(xs, ys, marker="o", color="black")

        # --- DESENHAR PONTOS ESPECIAIS ---
        for nome, (px, py) in self.PONTOS_ESPECIAIS.items():
            cor = "green" if nome == "Início" else \
                "blue" if nome == "Destino 1" else \
                "red"

            self.ax.scatter(px, py, color=cor, s=100, label=nome)

        # Mostrar legenda dos pontos importantes
        self.ax.legend(loc="upper left")

        self.ax.set_title("Visualização da Rota (10x10)")
        self.canvas.draw()

    # ---------------------------------------------------------
    def salvar_rota(self):
        if not self.pontos:
            messagebox.showwarning("Aviso", "Não há pontos para salvar. Crie uma rota primeiro.")
            return

        nome = simpledialog.askstring("Salvar", "Nome da rota (ex: linha_principal):")
        if not nome:
            return

        safe = sanitize_filename(nome)
        if not safe:
            messagebox.showerror("Erro", "Nome inválido para arquivo.")
            return

        caminho = os.path.join(PASTA_ROTAS, f"{safe}.txt")
        try:
            with open(caminho, "w") as f:
                for x, y in self.pontos:
                    f.write(f"{x},{y}\n")
        except Exception as e:
            messagebox.showerror("Erro ao salvar", f"Não foi possível salvar a rota:\n{e}")
            return

        self.atualizar_lista()
        messagebox.showinfo("OK", f"Rota '{safe}' salva em {PASTA_ROTAS}/{safe}.txt")

        self.modo_criacao = False       
        self.pontos = [(0, 0)]          
        self.x, self.y = 0, 0           
        self.atualizar_grafico()        

    # ---------------------------------------------------------
    def carregar_rota(self):
        selecionada = self.lista_rotas.get(tk.ACTIVE)
        if not selecionada:
            messagebox.showwarning("Erro", "Nenhuma rota selecionada.")
            return

        caminho = os.path.join(PASTA_ROTAS, selecionada)

        pontos = []
        try:
            with open(caminho, "r") as f:
                for linha in f:
                    if not linha.strip():
                        continue
                    x, y = linha.strip().split(",")
                    pontos.append((float(x), float(y)))
        except Exception as e:
            messagebox.showerror("Erro ao carregar", f"Não foi possível carregar a rota:\n{e}")
            return

        self.pontos = pontos
        self.x, self.y = pontos[-1]
        self.modo_criacao = False
        self.atualizar_grafico()

        messagebox.showinfo("Carregada", f"Rota '{selecionada}' carregada!")

    # ---------------------------------------------------------
    def enviar_rota(self):
        selecionada = self.lista_rotas.get(tk.ACTIVE)
        if not selecionada:
            messagebox.showwarning("Erro", "Selecione uma rota antes de adicionar à fila.")
            return

        # Adiciona na FILA FIFO
        rotas_para_envio.append(selecionada)

        messagebox.showinfo("Fila", 
            f"Rota '{selecionada}' adicionada à fila FIFO de envio.\n"
            f"Total na fila: {len(rotas_para_envio)}"
        )

    # ---------------------------------------------------------
    def atualizar_lista(self):
        self.lista_rotas.delete(0, tk.END)
        for arquivo in sorted(os.listdir(PASTA_ROTAS)):
            if arquivo.endswith(".txt"):
                self.lista_rotas.insert(tk.END, arquivo)
                
    def traduzir_rota_para_json(self, nome_arquivo):
        caminho = os.path.join(PASTA_ROTAS, nome_arquivo)

        lista_pontos = []

        try:
            with open(caminho, "r") as f:
                for linha in f:
                    if linha.strip():
                        x, y = linha.strip().split(",")
                        lista_pontos.append({"x": int(x), "y": int(y)})
        except Exception as e:
            print(f"Erro ao traduzir o arquivo: {e}")
            return None

        # Converter para JSON
        json_rota = json.dumps({
            "nome": nome_arquivo.replace(".txt", ""),
            "pontos": lista_pontos
        })

        return json_rota
    
    def enviar_proxima_da_fila(self):
        if not rotas_para_envio:
            print("[MQTT] Fila vazia. Nada para enviar.")
            return

        rota = rotas_para_envio.pop(0)
        print(f"[MQTT] Enviando rota: {rota}")
        
        # Traduz o txt para JSON
        json_rota = self.traduzir_rota_para_json(rota)
        if json_rota is None:
            print("[MQTT] Erro ao converter rota.")
            return

        # Envia para o caminhão
        self.mqtt_client.publish("hovercraft/traj", json_rota)

        messagebox.showinfo("MQTT", f"Rota enviada via MQTT: {rota}")
    
    def on_connect(self, client, userdata, flags, rc):
        print("MQTT conectado. Código:", rc)
        client.subscribe("hovercraft/soli_traj")
        
    def on_message(self, client, userdata, msg):
        print(f"[MQTT] Mensagem recebida: {msg.payload.decode()} no tópico {msg.topic}")

        if msg.topic == "hovercraft/soli_traj":
            if msg.payload.decode() == "PEDIR":
                self.enviar_proxima_da_fila()


if __name__ == "__main__":
    root = tk.Tk()
    app = CriadorRotaSetas(root)
    root.mainloop()