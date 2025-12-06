# Setup Guide - Physical AI & Humanoid Robotics Textbook

This guide will help you set up and run the Physical AI textbook with the RAG chatbot locally.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Running the Application](#running-the-application)
5. [Deployment](#deployment)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

### Required Software

- **Node.js**: Version 18.x or higher
  - Download from [nodejs.org](https://nodejs.org/)
  - Verify installation: `node --version`

- **Python**: Version 3.8 or higher
  - Download from [python.org](https://www.python.org/)
  - Verify installation: `python --version`

- **Git**: For version control
  - Download from [git-scm.com](https://git-scm.com/)

### Required Accounts

1. **OpenAI Account**
   - Sign up at [platform.openai.com](https://platform.openai.com/)
   - Create an API key from the dashboard
   - Note: You'll need billing enabled for API usage

2. **Qdrant Cloud Account**
   - Sign up at [cloud.qdrant.io](https://cloud.qdrant.io/)
   - Free tier is sufficient for this project
   - Create a cluster and note the URL and API key

## Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/your-username/Hackhathon-ai-book.git
cd Hackhathon-ai-book
```

### Step 2: Install Docusaurus Dependencies

```bash
cd physical-ai-book
npm install
```

This will install all required Node.js packages for the Docusaurus site.

### Step 3: Install Python Dependencies

```bash
cd ../rag-backend
pip install -r requirements.txt
```

Or use a virtual environment (recommended):

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Configuration

### Step 1: Configure RAG Backend

Create a `.env` file in the `rag-backend` directory:

```bash
cd rag-backend
cp .env.example .env
```

Edit the `.env` file with your credentials:

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-your-openai-api-key-here

# Qdrant Configuration
QDRANT_HOST=https://your-cluster-id.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here

# Optional: RAG API URL (for ingestion)
RAG_API_URL=http://localhost:8000
```

### Step 2: Initialize Qdrant Collection

Run the setup script to create the vector database collection:

```bash
cd rag-backend
python setup_qdrant.py
```

Expected output:
```
Collection 'physical_ai_book' created.
Qdrant client setup complete.
```

### Step 3: Configure Docusaurus (Optional for GitHub Pages)

If you plan to deploy to GitHub Pages, edit `physical-ai-book/docusaurus.config.ts`:

```typescript
url: 'https://your-github-username.github.io',
baseUrl: '/Hackhathon-ai-book/',
organizationName: 'your-github-username',
projectName: 'Hackhathon-ai-book',
```

## Running the Application

### Step 1: Start the RAG Backend

Open a terminal and run:

```bash
cd rag-backend
python main.py
```

Expected output:
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

The API will be available at `http://localhost:8000`

Test the API:
```bash
curl http://localhost:8000/health
```

### Step 2: Ingest Book Content

Open a new terminal and run the ingestion script:

```bash
cd scripts
python ingest.py
```

This will:
- Read all markdown files from the docs directory
- Chunk them into manageable segments
- Generate embeddings using OpenAI
- Upload them to Qdrant

Expected output:
```
Starting document ingestion from: /path/to/physical-ai-book/docs
API endpoint: http://localhost:8000
✓ API is healthy
Found 12 markdown files
Processing: chapter1.md
✓ Upserted chunk 0_abc123...
...
Ingestion complete!
Total chunks processed: 147
Successful upserts: 147
```

### Step 3: Start Docusaurus

Open a new terminal and run:

```bash
cd physical-ai-book
npm start
```

Expected output:
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

The book will open in your default browser at `http://localhost:3000`

### Step 4: Test the Chatbot

1. Navigate to any page in the book
2. Click the floating chat button (💬) in the bottom-right corner
3. Try asking: "What is Physical AI?"
4. You should see an AI-powered response with source citations

## Deployment

### Deploy to GitHub Pages

#### Option 1: Automatic Deployment (Recommended)

1. Push your code to GitHub:
   ```bash
   git add .
   git commit -m "Initial commit"
   git push origin main
   ```

2. GitHub Actions will automatically build and deploy to GitHub Pages

3. Enable GitHub Pages in repository settings:
   - Go to Settings > Pages
   - Source: Deploy from a branch
   - Branch: gh-pages
   - Click Save

#### Option 2: Manual Deployment

```bash
cd physical-ai-book

# Set your Git user (if not already set)
git config user.name "Your Name"
git config user.email "your.email@example.com"

# Build and deploy
npm run build
npm run deploy
```

### Deploy RAG Backend to Render

1. Sign up at [render.com](https://render.com/)

2. Create a new Web Service:
   - Connect your GitHub repository
   - Root Directory: `rag-backend`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`

3. Add environment variables in Render dashboard:
   - `OPENAI_API_KEY`
   - `QDRANT_HOST`
   - `QDRANT_API_KEY`

4. Update chatbot API URL in `physical-ai-book/src/theme/Root.tsx`:
   ```typescript
   <Chatbot apiUrl="https://your-app.onrender.com" />
   ```

5. Rebuild and redeploy Docusaurus

### Deploy RAG Backend to Vercel (Alternative)

1. Install Vercel CLI:
   ```bash
   npm install -g vercel
   ```

2. Deploy from rag-backend directory:
   ```bash
   cd rag-backend
   vercel
   ```

3. Add environment variables via Vercel dashboard

4. Update chatbot API URL as described above

## Troubleshooting

### Common Issues

#### Issue: "Module not found" errors in Docusaurus

**Solution:**
```bash
cd physical-ai-book
rm -rf node_modules package-lock.json
npm install
```

#### Issue: "Collection does not exist" in RAG backend

**Solution:**
```bash
cd rag-backend
python setup_qdrant.py
```

#### Issue: Chatbot shows "Error querying RAG"

**Checklist:**
1. Is the RAG backend running? Check `http://localhost:8000/health`
2. Is Qdrant configured correctly? Check `.env` file
3. Have you ingested the documents? Run `python scripts/ingest.py`
4. Check browser console for detailed error messages

#### Issue: "OpenAI API key not found"

**Solution:**
1. Verify `.env` file exists in `rag-backend/`
2. Check that `OPENAI_API_KEY` is set correctly
3. Make sure there are no extra spaces or quotes
4. Restart the backend server

#### Issue: Build fails with TypeScript errors

**Solution:**
```bash
cd physical-ai-book
npm run build -- --no-minify
```

This will show more detailed error messages.

### Getting Help

If you encounter issues:

1. Check the [GitHub Issues](https://github.com/your-username/Hackhathon-ai-book/issues)
2. Review the logs for error messages
3. Use the chatbot in the book to ask questions
4. Create a new issue with:
   - Steps to reproduce
   - Error messages
   - Your environment (OS, Node version, Python version)

## Performance Tips

### Optimize Qdrant Performance

- Use the free tier for development
- For production, consider upgrading to a paid plan
- Adjust `chunk_size` in `utils.py` based on your content (current: 800 chars)

### Optimize OpenAI Costs

- Use `text-embedding-3-small` instead of `-large` for embeddings
- Cache common queries if implementing on backend
- Set reasonable rate limits

### Speed Up Docusaurus Build

- Use `npm run build -- --bundleAnalyzer` to identify large dependencies
- Enable webpack cache in production builds
- Consider using a CDN for static assets

## Next Steps

After setup:

1. **Customize Content**: Edit chapters in `physical-ai-book/docs/chapters/`
2. **Customize Chatbot**: Modify `physical-ai-book/src/components/Chatbot.tsx`
3. **Add Features**: Explore Phase 7 advanced features in `tasks.md`
4. **Deploy**: Follow deployment instructions above

## Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenAI API Documentation](https://platform.openai.com/docs/)

---

**Questions?** Open an issue or use the chatbot feature in the deployed book!
