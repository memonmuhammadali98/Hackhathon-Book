# Physical AI & Humanoid Robotics Textbook 🤖

An interactive, AI-powered textbook for learning Physical AI and Humanoid Robotics with built-in RAG (Retrieval-Augmented Generation) chatbot.

## 🌟 Features

- **Comprehensive Content**: 12+ chapters covering Physical AI fundamentals, ROS 2, simulation environments, VSLAM, and Navigation2
- **Interactive RAG Chatbot**: Ask questions about the book content and get AI-powered answers with source citations
- **Text Selection Query**: Select any text in the book and ask questions specifically about it
- **Modern Tech Stack**: Built with Docusaurus 3, FastAPI, Qdrant, and OpenAI
- **Responsive Design**: Works seamlessly on desktop and mobile devices

## 📚 Book Contents

### Part 1: Foundational Concepts
- Chapter 1: Introduction to Physical AI and Embodied Intelligence
- Chapter 2: Humanoid Robotics Fundamentals
- Chapter 3: Getting Started with ROS 2

### Part 2: Simulation & Control
- Chapters on Gazebo, Unity, NVIDIA Isaac
- Practical simulation environments
- Capstone humanoid robot project

### Part 3: Advanced Topics
- Visual SLAM (VSLAM)
- Navigation2 (Nav2)
- Advanced perception and control

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ and npm
- Python 3.8+
- OpenAI API key
- Qdrant Cloud account (free tier)

### 1. Install Dependencies

```bash
# Install Docusaurus dependencies
cd physical-ai-book
npm install

# Install RAG backend dependencies
cd ../rag-backend
pip install -r requirements.txt
```

### 2. Configure Environment

Create `rag-backend/.env` file:

```bash
OPENAI_API_KEY=your_openai_api_key
QDRANT_HOST=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
```

### 3. Setup Qdrant Collection

```bash
cd rag-backend
python setup_qdrant.py
```

### 4. Start RAG Backend

```bash
cd rag-backend
python main.py
```

The API will be available at `http://localhost:8000`

### 5. Ingest Book Content

```bash
cd scripts
python ingest.py
```

### 6. Start Docusaurus

```bash
cd physical-ai-book
npm start
```

Visit `http://localhost:3000` to view the book!

## 🏗️ Project Structure

```
.
├── physical-ai-book/          # Docusaurus book
│   ├── docs/                  # Markdown chapters
│   │   ├── chapters/         # Main book chapters
│   │   ├── intro/            # Introduction
│   │   └── ...
│   ├── src/
│   │   ├── components/       # React components
│   │   │   └── Chatbot.tsx  # RAG chatbot UI
│   │   └── theme/           # Theme customization
│   └── docusaurus.config.ts # Docusaurus configuration
│
├── rag-backend/              # FastAPI RAG backend
│   ├── main.py              # API endpoints
│   ├── utils.py             # Document chunking utilities
│   ├── setup_qdrant.py      # Qdrant setup script
│   └── requirements.txt     # Python dependencies
│
├── scripts/
│   └── ingest.py            # Document ingestion script
│
└── specs/                   # Specification documents
    └── 001-physical-ai-robotics-book/
        ├── spec.md
        ├── plan.md
        └── tasks.md
```

## 🛠️ Technologies Used

### Frontend
- **Docusaurus 3**: Static site generator for documentation
- **React 18**: UI components
- **TypeScript**: Type-safe development

### Backend
- **FastAPI**: Modern Python web framework
- **Qdrant**: Vector database for semantic search
- **OpenAI Embeddings**: text-embedding-3-small model

### Infrastructure
- **GitHub Pages**: Static site hosting
- **GitHub Actions**: CI/CD pipeline
- **Render/Vercel**: Backend API hosting

## 📖 Using the RAG Chatbot

### Ask About the Whole Book
1. Click the chat button (💬) in the bottom-right corner
2. Type your question
3. Click "📚 Ask" to search across all book content

### Ask About Selected Text
1. Select any text in the book
2. Open the chatbot
3. Type your question
4. Click "✂️ Ask Selection" to query specifically about the selected text

### Features
- **Source Citations**: See which chapters and sections the answer comes from
- **Context-Aware**: Understands the book's structure and content
- **Semantic Search**: Finds relevant content even if exact words don't match

## 🚢 Deployment

### Deploy Book to GitHub Pages

1. Update `docusaurus.config.ts` with your GitHub username and repo
2. Push to main branch
3. GitHub Actions will automatically build and deploy

Or manually:

```bash
cd physical-ai-book
npm run build
npm run deploy
```

### Deploy RAG Backend to Render

1. Create a new Web Service on Render
2. Connect your GitHub repository
3. Set build command: `pip install -r rag-backend/requirements.txt`
4. Set start command: `cd rag-backend && uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (OPENAI_API_KEY, QDRANT_HOST, QDRANT_API_KEY)

## 📝 Development

### Add New Chapters

1. Create markdown file in `physical-ai-book/docs/chapters/`
2. Follow the existing chapter structure
3. Run ingestion script to add to RAG system:

```bash
cd scripts
python ingest.py
```

### Customize Chatbot

Edit `physical-ai-book/src/components/Chatbot.tsx` to:
- Change styling
- Modify prompts
- Add new features
- Customize responses

## 🧪 Testing

### Test RAG Backend

```bash
# Health check
curl http://localhost:8000/health

# Test query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "top_k": 3}'
```

### Build Book Locally

```bash
cd physical-ai-book
npm run build
npm run serve
```

## 📄 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📧 Contact

For questions or feedback, please open an issue on GitHub.

---

Built with ❤️ for the Physical AI and Robotics community
Developed by **Muhammad Ali**
npm i