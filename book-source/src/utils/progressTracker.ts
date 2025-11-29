/**
 * Progress Tracking Utility
 *
 * Manages student progress using localStorage for client-side persistence.
 * Tracks lesson completion, exercise completion, and quiz scores.
 */

export interface ExerciseProgress {
  exerciseId: string;
  completed: boolean;
  attempts: number;
  lastAttempt: string; // ISO date string
  passed: boolean;
}

export interface LessonProgress {
  lessonId: string;
  completed: boolean;
  startedAt?: string; // ISO date string
  completedAt?: string; // ISO date string
  exercises: Record<string, ExerciseProgress>;
  timeSpent: number; // minutes
}

export interface ChapterProgress {
  chapterId: number;
  lessonsCompleted: number;
  totalLessons: number;
  quizScore?: number; // 0-100
  quizAttempts: number;
  completed: boolean;
}

export interface StudentProgress {
  version: string; // for schema migrations
  lastUpdated: string; // ISO date string
  lessons: Record<string, LessonProgress>;
  chapters: Record<number, ChapterProgress>;
  totalTimeSpent: number; // minutes
}

const STORAGE_KEY = 'studentProgress';
const CURRENT_VERSION = '1.0.0';

/**
 * Initialize or load student progress from localStorage
 */
export function loadProgress(): StudentProgress {
  if (typeof window === 'undefined') {
    // Server-side rendering - return empty progress
    return createEmptyProgress();
  }

  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      return createEmptyProgress();
    }

    const progress: StudentProgress = JSON.parse(stored);

    // Handle version migrations if needed
    if (progress.version !== CURRENT_VERSION) {
      return migrateProgress(progress);
    }

    return progress;
  } catch (error) {
    console.error('Error loading progress:', error);
    return createEmptyProgress();
  }
}

/**
 * Save student progress to localStorage
 */
export function saveProgress(progress: StudentProgress): boolean {
  if (typeof window === 'undefined') {
    return false;
  }

  try {
    progress.lastUpdated = new Date().toISOString();
    localStorage.setItem(STORAGE_KEY, JSON.stringify(progress));
    return true;
  } catch (error) {
    console.error('Error saving progress:', error);
    return false;
  }
}

/**
 * Mark a lesson as started
 */
export function startLesson(lessonId: string): void {
  const progress = loadProgress();

  if (!progress.lessons[lessonId]) {
    progress.lessons[lessonId] = {
      lessonId,
      completed: false,
      startedAt: new Date().toISOString(),
      exercises: {},
      timeSpent: 0,
    };
    saveProgress(progress);
  }
}

/**
 * Mark a lesson as completed
 */
export function completeLesson(lessonId: string): void {
  const progress = loadProgress();

  if (!progress.lessons[lessonId]) {
    startLesson(lessonId);
  }

  progress.lessons[lessonId].completed = true;
  progress.lessons[lessonId].completedAt = new Date().toISOString();

  saveProgress(progress);
}

/**
 * Update exercise progress
 */
export function updateExerciseProgress(
  lessonId: string,
  exerciseId: string,
  passed: boolean
): void {
  const progress = loadProgress();

  if (!progress.lessons[lessonId]) {
    startLesson(lessonId);
  }

  const lesson = progress.lessons[lessonId];
  const exercise = lesson.exercises[exerciseId] || {
    exerciseId,
    completed: false,
    attempts: 0,
    lastAttempt: new Date().toISOString(),
    passed: false,
  };

  exercise.attempts += 1;
  exercise.lastAttempt = new Date().toISOString();
  exercise.passed = passed;
  exercise.completed = passed;

  lesson.exercises[exerciseId] = exercise;
  saveProgress(progress);
}

/**
 * Update chapter progress
 */
export function updateChapterProgress(
  chapterId: number,
  totalLessons: number
): void {
  const progress = loadProgress();

  // Count completed lessons for this chapter
  const completedLessons = Object.values(progress.lessons).filter(
    lesson => {
      // Extract chapter number from lessonId (format: "chapter-XX-lesson-YY")
      const match = lesson.lessonId.match(/chapter-(\d+)-/);
      return match && parseInt(match[1]) === chapterId && lesson.completed;
    }
  ).length;

  progress.chapters[chapterId] = {
    chapterId,
    lessonsCompleted: completedLessons,
    totalLessons,
    quizAttempts: progress.chapters[chapterId]?.quizAttempts || 0,
    quizScore: progress.chapters[chapterId]?.quizScore,
    completed: completedLessons === totalLessons,
  };

  saveProgress(progress);
}

/**
 * Update quiz score for a chapter
 */
export function updateQuizScore(
  chapterId: number,
  score: number
): void {
  const progress = loadProgress();

  if (!progress.chapters[chapterId]) {
    progress.chapters[chapterId] = {
      chapterId,
      lessonsCompleted: 0,
      totalLessons: 0,
      quizAttempts: 0,
      completed: false,
    };
  }

  const chapter = progress.chapters[chapterId];
  chapter.quizScore = Math.max(chapter.quizScore || 0, score);
  chapter.quizAttempts += 1;

  saveProgress(progress);
}

/**
 * Get overall progress summary
 */
export function getProgressSummary(): {
  lessonsCompleted: number;
  chaptersCompleted: number;
  totalTimeSpent: number;
  quizzesCompleted: number;
} {
  const progress = loadProgress();

  return {
    lessonsCompleted: Object.values(progress.lessons).filter(l => l.completed).length,
    chaptersCompleted: Object.values(progress.chapters).filter(c => c.completed).length,
    totalTimeSpent: progress.totalTimeSpent,
    quizzesCompleted: Object.values(progress.chapters).filter(c => c.quizScore !== undefined).length,
  };
}

/**
 * Export progress as JSON for backup/portability
 */
export function exportProgress(): string {
  const progress = loadProgress();
  return JSON.stringify(progress, null, 2);
}

/**
 * Import progress from JSON
 */
export function importProgress(jsonData: string): boolean {
  try {
    const progress: StudentProgress = JSON.parse(jsonData);

    // Validate structure
    if (!progress.version || !progress.lessons || !progress.chapters) {
      throw new Error('Invalid progress data structure');
    }

    saveProgress(progress);
    return true;
  } catch (error) {
    console.error('Error importing progress:', error);
    return false;
  }
}

/**
 * Clear all progress (with confirmation)
 */
export function clearProgress(): boolean {
  if (typeof window === 'undefined') {
    return false;
  }

  try {
    localStorage.removeItem(STORAGE_KEY);
    return true;
  } catch (error) {
    console.error('Error clearing progress:', error);
    return false;
  }
}

/**
 * Create empty progress structure
 */
function createEmptyProgress(): StudentProgress {
  return {
    version: CURRENT_VERSION,
    lastUpdated: new Date().toISOString(),
    lessons: {},
    chapters: {},
    totalTimeSpent: 0,
  };
}

/**
 * Migrate progress from older versions
 */
function migrateProgress(oldProgress: StudentProgress): StudentProgress {
  // For now, just update the version
  // Add migration logic here if schema changes in the future
  return {
    ...oldProgress,
    version: CURRENT_VERSION,
  };
}

/**
 * Check if a lesson is completed
 */
export function isLessonCompleted(lessonId: string): boolean {
  const progress = loadProgress();
  return progress.lessons[lessonId]?.completed || false;
}

/**
 * Get chapter completion percentage
 */
export function getChapterCompletion(chapterId: number): number {
  const progress = loadProgress();
  const chapter = progress.chapters[chapterId];

  if (!chapter || chapter.totalLessons === 0) {
    return 0;
  }

  return Math.round((chapter.lessonsCompleted / chapter.totalLessons) * 100);
}
